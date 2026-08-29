// Migrated from Project 2 (FILES PROJECT 2/tests/components/mission_control_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Structurally adapted for Project 3's DI/module layout (see
// §3/§7.2): Project 2 injected IDroneControl directly into MissionControlImpl and mocked
// it to unit-test MissionControlImpl in isolation, returning a fixed DroneStepResult per
// step() call. Project 3's MissionControlImpl has no such external injection seam — it
// builds its own DroneControlImpl internally from MissionControlDependencies — so every
// test below instead mocks the *next layer down* (IMappingAlgorithm, plus
// ILidar/IGPS/IDroneMovement/IMutableMap3D to satisfy the real DroneControlImpl's other
// dependencies) and drives runMission() through a real DroneControlImpl. Concretely: a
// mocked DroneStepResult{Completed, "mapping finished"} becomes a mocked
// MappingStepCommand{.status = Finished} from IMappingAlgorithm (DroneControlImpl maps
// Finished -> Completed/"mapping finished" itself, per its unchanged step() logic — see
// PROJ2_TESTS_PLAN.md §7.2); a mocked DroneStepResult{Error, "..."} becomes a movement
// command whose mocked IDroneMovement call fails with that message (DroneControlImpl maps
// a failed movement to Error/that message). This turns these from pure interaction tests
// (verify a mock was called N times) into small state-based tests (verify the resulting
// MissionRunResult given a scripted IMappingAlgorithm/IDroneMovement mock) -- a bigger
// shift than a rename, per §7.2. Every test's original intent, inputs (max_steps, output
// path), and assertions are otherwise preserved 1:1.

#include <MissionControl/MissionControlImpl.h>

#include "mocks/GMockIDroneMovement.h"
#include "mocks/GMockIGPS.h"
#include "mocks/GMockILidar.h"
#include "mocks/GMockIMappingAlgorithm.h"
#include "mocks/GMockIMutableMap3D.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace common;
using namespace common::types;
using namespace mission_control_322889890_315113738;
using ::testing::_;
using ::testing::Eq;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

namespace {

DroneConfigData droneConfig() {
    DroneConfigData config;
    config.radius = 5.0 * isq::length[cm];
    config.max_rotate = 45.0 * horizontal_angle[deg];
    config.max_advance = 50.0 * isq::length[cm];
    config.max_elevate = 40.0 * isq::length[cm];
    return config;
}

MissionConfigData missionConfig(std::size_t max_steps) {
    MissionConfigData config;
    config.max_steps = max_steps;
    config.gps_resolution = 10.0 * isq::length[cm];
    config.output_mapping_resolution_factor = 1.0;
    return config;
}

LidarConfigData lidarConfig() {
    LidarConfigData config;
    config.z_min = 20.0 * isq::length[cm];
    config.z_max = 120.0 * isq::length[cm];
    config.d = 2.5 * isq::length[cm];
    config.fov_circles = 1;
    return config;
}

// MappingStepCommand equivalents of the Project 2 fixture's DroneStepResult constants --
// what the mocked algorithm must return so the real DroneControlImpl produces the
// corresponding DroneStepStatus (see the file banner above).
MappingStepCommand workingCommand() {
    MappingStepCommand command;
    command.status = AlgorithmStatus::Working;
    // Hover so this stays a legitimate "nothing happens" step rather than a faulty NOOP
    // (Optional Common-Issues row 4: Working with neither movement nor scan). gps_ here always
    // returns a default-constructed Position3D (never stubbed with a per-test sequence in this
    // file), so Hover's row-12 post-movement re-read always matches trivially.
    MovementCommand hover;
    hover.type = MovementCommandType::Hover;
    command.movement = hover;
    return command;
}

MappingStepCommand finishedCommand() {
    MappingStepCommand command;
    command.status = AlgorithmStatus::Finished;
    return command;
}

MappingStepCommand unmappableVoxelsCommand() {
    MappingStepCommand command;
    command.status = AlgorithmStatus::FinishedWithUnmappableVoxels;
    return command;
}

// A scan-only command whose mocked ILidar::scan() call (set up by the caller, via
// Throw(std::runtime_error(...)) -- see the file's using-declarations) throws, driving
// DroneControlImpl::step() to a *returned* DroneStepStatus::Error with that message. A movement
// exception is no longer a fit for these Row-9 (Error-is-non-terminal) tests: a real wall
// collision is now a fatal simulation-run failure that propagates out of step() unchanged, not a
// DroneStepStatus::Error MissionControl can log and continue past. lidar_.scan() throwing (e.g.
// its own map access failing) is a genuine, still-immediate, still-unretried returned-Error path
// instead.
MappingStepCommand scanCommand() {
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    return command;
}

// Generous, effectively unbounded map used as the fixture's default MapConfig. Without it, an
// unstubbed output_map_.getMapConfig() would return a default-constructed MapConfig (all-zero
// boundaries), which DroneControlImpl's out-of-bounds handling (Optional Common-Issues row 11)
// would treat the fixture's default (0,0,0) GPS reading as already out of bounds. Tests here only
// script IMappingAlgorithm/ILidar/IDroneMovement outcomes; none of them mean to exercise
// out-of-bounds handling itself, so the bounds just need to be wide enough to never bind.
MapConfig unboundedMapConfig() {
    MapConfig config;
    config.boundaries = MappingBounds{
        -100000.0 * x_extent[cm], 100000.0 * x_extent[cm], -100000.0 * y_extent[cm],
        100000.0 * y_extent[cm], -100000.0 * z_extent[cm], 100000.0 * z_extent[cm]};
    return config;
}

// A real (half-open) bounds check against `bounds`, used as the test double's isInBounds()
// behavior -- DroneControlImpl treats output_map_.isInBounds() as the sole authority on
// legality, so an unstubbed default (NiceMock returns false) would make it treat every
// Advance/Elevate as out of bounds. Captures `bounds` by value rather than calling back into
// output_map_.getMapConfig(), so it never perturbs a test's own call-count expectations on that
// method.
[[nodiscard]] auto isInBoundsFor(const MappingBounds& bounds) {
    return [bounds](const Position3D& pos) {
        return pos.x >= bounds.min_x && pos.x < bounds.max_x && pos.y >= bounds.min_y &&
               pos.y < bounds.max_y && pos.z >= bounds.min_height && pos.z < bounds.max_height;
    };
}

class MissionControl : public ::testing::Test {
protected:
    NiceMock<test::GMockILidar> lidar_;
    NiceMock<test::GMockIGPS> gps_;
    NiceMock<test::GMockIDroneMovement> movement_;
    NiceMock<test::GMockIMutableMap3D> output_map_;
    NiceMock<test::GMockIMappingAlgorithm> algorithm_{
        MappingAlgorithmDependencies{missionConfig(1), lidarConfig(), droneConfig(), output_map_}};

    void SetUp() override {
        ON_CALL(output_map_, getMapConfig()).WillByDefault(Return(unboundedMapConfig()));
        ON_CALL(output_map_, isInBounds(_)).WillByDefault(Invoke(isInBoundsFor(unboundedMapConfig().boundaries)));
    }

    std::unique_ptr<MissionControlImpl> makeMissionControl(std::size_t max_steps,
                                                           const std::filesystem::path& output_map_file = "out.npy",
                                                           bool verbose = false) {
        return std::make_unique<MissionControlImpl>(MissionControlDependencies{
            missionConfig(max_steps), droneConfig(), lidar_, gps_, movement_, output_map_, algorithm_,
            output_map_file, verbose});
    }
};

} // namespace

TEST_F(MissionControl, CompletedStatusStopsLoopAndReportsCompleted) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
}

TEST_F(MissionControl, MaxStepsReachedWithoutTerminalStatusReportsMaxSteps) {
    constexpr std::size_t kMaxSteps = 3;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(kMaxSteps).WillRepeatedly(Return(workingCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(kMaxSteps);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, kMaxSteps);
}

TEST_F(MissionControl, ErrorStatusIsLoggedAndMissionContinuesToCompleted) {
    // Row 9 (Optional Common-Issues): a returned DroneStepStatus::Error is non-terminal at
    // MissionControl level -- it is logged/recorded, but the mission loop keeps going and a
    // later Completed still resolves the run as Completed.
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked by obstacle")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_CONTROL_ERROR");
    EXPECT_EQ(result.errors[0].message, "blocked by obstacle");
}

TEST_F(MissionControl, ErrorThenContinueThenCompletedRunsAllThreeSteps) {
    // Confirms MissionControl genuinely resumes the loop after an Error -- not just that it
    // tolerates one immediately followed by a terminal Completed.
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked by obstacle")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "blocked by obstacle");
}

TEST_F(MissionControl, MultipleErrorsAreAllRetainedInOrderBeforeCompleted) {
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Throw(std::runtime_error("first obstacle")))
        .WillOnce(Throw(std::runtime_error("second obstacle")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3u);
    ASSERT_EQ(result.errors.size(), 2u);
    EXPECT_EQ(result.errors[0].message, "first obstacle");
    EXPECT_EQ(result.errors[1].message, "second obstacle");
}

TEST_F(MissionControl, ErrorsUntilMaxStepsReportsMaxStepsNotError) {
    // Every attempted step returns Error, and the budget runs out without a Completed: the
    // final status must be MaxSteps (never Error), with one step() call per max_steps and every
    // error retained.
    constexpr std::size_t kMaxSteps = 3;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(kMaxSteps).WillRepeatedly(Return(scanCommand()));
    EXPECT_CALL(lidar_, scan(_)).Times(kMaxSteps).WillRepeatedly(Throw(std::runtime_error("blocked")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(kMaxSteps);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, kMaxSteps);
    ASSERT_EQ(result.errors.size(), kMaxSteps);
}

TEST_F(MissionControl, ErrorOnFinalAvailableStepReportsMaxStepsAndRetainsError) {
    constexpr std::size_t kMaxSteps = 2;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(scanCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked at the end")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(kMaxSteps);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, kMaxSteps);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "blocked at the end");
}

TEST_F(MissionControl, OutputMapSavedExactlyOnceWhenCompleted) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(_)).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    (void)mission_control->runMission();
}

TEST_F(MissionControl, OutputMapSavedExactlyOnceWhenErrorOccursThenCompletes) {
    // A recoverable DroneStepStatus::Error must not disturb the normal final output handling.
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked by obstacle")));
    EXPECT_CALL(output_map_, save(_)).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    (void)mission_control->runMission();
}

TEST_F(MissionControl, OutputMapSavedExactlyOnceWhenMaxStepsReached) {
    constexpr std::size_t kMaxSteps = 3;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(kMaxSteps).WillRepeatedly(Return(workingCommand()));
    EXPECT_CALL(output_map_, save(_)).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(kMaxSteps);
    (void)mission_control->runMission();
}

TEST_F(MissionControl, StepsCountReflectsActualNumberOfStepCallsNotMaxSteps) {
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(finishedCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.steps, 3u);
}

TEST_F(MissionControl, DroneControlExceptionStopsMissionAndPreservesCompletedStepCount) {
    // Step 1 succeeds normally. On the next step, the Algorithm repeatedly returns
    // a faulty NOOP, causing DroneControl to exhaust its retries and throw.
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(MappingStepCommand{}))
        .WillOnce(Return(MappingStepCommand{}))
        .WillOnce(Return(MappingStepCommand{}));

    const std::unique_ptr<MissionControlImpl> mission_control =
        makeMissionControl(10);

    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u);

    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_CONTROL_EXCEPTION");
    EXPECT_NE(
        result.errors[0].message.find("invalid or no-op command"),
        std::string::npos);
}

TEST_F(MissionControl, UnmappableVoxelsCompletedStatusStillReportsCompletedButAddsError) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(unmappableVoxelsCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_FALSE(result.errors.empty());
}

TEST_F(MissionControl, PlainCompletedDoesNotAddSpuriousError) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_TRUE(result.errors.empty());
}

// ── max_steps boundary ───────────────────────────────────────────────────────

TEST_F(MissionControl, MaxStepsZeroNeverCallsDroneControlButStillReportsMaxStepsAndSaves) {
    // `while (steps < mission_.max_steps)`: with max_steps == 0 the loop body must never execute
    // at all. A bug that flips the comparison (e.g. `<=`) or initializes status to something
    // other than MaxSteps would call step() (and therefore nextStep()) at least once or report
    // the wrong status here.
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(0);
    EXPECT_CALL(output_map_, save(_)).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(0);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 0u);
    EXPECT_TRUE(result.errors.empty());
}

// ── output contract details ──────────────────────────────────────────────────

TEST_F(MissionControl, OutputMapIsSavedToTheExactConstructorProvidedPath) {
    // The existing OutputMapSavedExactlyOnce* tests only check the call *count*; a bug that saves
    // to a wrong/default path (e.g. a stray literal instead of forwarding output_map_file_) would
    // still pass those. Pin down the actual argument too.
    const std::filesystem::path expected_path = "mission_control_test/specific_output.npy";
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(Eq(expected_path))).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10, expected_path);
    (void)mission_control->runMission();
}

TEST_F(MissionControl, ErrorStatusReportsStepCountAndDroneControlErrorCode) {
    // ErrorStatusIsLoggedAndMissionContinuesToCompleted checks only the error message; the
    // "DRONE_CONTROL_ERROR" code and the steps count (++steps happens before the status check, so
    // an immediate Error must still report steps == 1 for that attempt, not 0) are separate
    // output-contract details that a bug could get wrong independently of the message.
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked by obstacle")));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.steps, 2u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_CONTROL_ERROR");
}

TEST_F(MissionControl, UnmappableVoxelsErrorHasExpectedCodeAndMessage) {
    // UnmappableVoxelsCompletedStatusStillReportsCompletedButAddsError only checks errors is
    // non-empty; pin down the actual code/message contract so a bug that pushes a generic or
    // wrong ErrorRef here still fails.
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(unmappableVoxelsCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "UNMAPPABLE_VOXELS_REMAINING");
    EXPECT_EQ(result.errors[0].message, "mapping finished with unmappable voxels remaining");
}

// --- Phase 2B fix 2: output_map_.save() failure in MissionControlImpl::runMission() --------
//
// The mission's outcome (status/steps/errors) is already fully determined by the time save() is
// called; a save failure must not erase that outcome by propagating out of runMission() -- it is
// appended as an additional OUTPUT_MAP_SAVE_FAILED error and the already-determined result is
// still returned normally.
// ── -verbose output ──────────────────────────────────────────────────────────
//
// output_map_.save() is mocked in this fixture (it never actually writes out.npy), so any file
// found in a run's output directory afterwards can only be a verbose diagnostics file
// MissionControlImpl itself created -- these tests count directory entries rather than
// hardcoding the log's name, staying agnostic to that naming detail.

namespace {

// A fresh, empty per-test directory under tests/component/test_output/mission_control_test, so
// runs from different tests never share or collide over output files.
[[nodiscard]] std::filesystem::path freshOutputDir(const std::string& test_name) {
    const std::filesystem::path dir =
        std::filesystem::path("tests/component/test_output/mission_control_test") / test_name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

[[nodiscard]] std::size_t countFilesIn(const std::filesystem::path& dir) {
    std::size_t count = 0;
    for ([[maybe_unused]] const auto& entry : std::filesystem::directory_iterator(dir)) {
        ++count;
    }
    return count;
}

// Reads every regular file directly under `dir` and concatenates their contents -- used to
// inspect a verbose log's content without hardcoding its filename.
[[nodiscard]] std::string readAllFilesIn(const std::filesystem::path& dir) {
    std::string contents;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::ifstream file(entry.path());
        contents.append(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }
    return contents;
}

} // namespace

TEST_F(MissionControl, WithoutVerboseFlagNoOutputFileIsCreated) {
    const std::filesystem::path dir = freshOutputDir("without_verbose_flag");
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control =
        makeMissionControl(10, dir / "out.npy", /*verbose=*/false);
    (void)mission_control->runMission();

    EXPECT_EQ(countFilesIn(dir), 0u);
}

TEST_F(MissionControl, WithVerboseFlagAnOutputFileIsCreatedWithMissionDetails) {
    const std::filesystem::path dir = freshOutputDir("with_verbose_flag");
    const std::filesystem::path output_map_file = dir / "out.npy";
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(finishedCommand()));

    const std::unique_ptr<MissionControlImpl> mission_control =
        makeMissionControl(10, output_map_file, /*verbose=*/true);
    (void)mission_control->runMission();

    ASSERT_EQ(countFilesIn(dir), 1u);
    const std::string contents = readAllFilesIn(dir);
    EXPECT_NE(contents.find("mission started"), std::string::npos);
    EXPECT_NE(contents.find("max_steps=10"), std::string::npos);
    EXPECT_NE(contents.find("mission finished"), std::string::npos);
    EXPECT_NE(contents.find("steps=2"), std::string::npos);
    EXPECT_NE(contents.find("Completed"), std::string::npos);
    // The final summary also carries the run's output map path and the terminal message that
    // actually ended it (here, DroneControlImpl's plain-Finished message).
    EXPECT_NE(contents.find("output_map=" + output_map_file.string()), std::string::npos);
    EXPECT_NE(contents.find("completion_reason=mapping finished"), std::string::npos);
    // Logging is event-based, not per-step: a short, error-free 2-step run must not produce a
    // "step N" line -- those only appear for meaningful events (errors/exceptions/checkpoints).
    EXPECT_EQ(contents.find("step "), std::string::npos);
}

TEST_F(MissionControl, VerboseFlagLogsMeaningfulErrorsAsTheyOccurNotEveryStep) {
    const std::filesystem::path dir = freshOutputDir("verbose_errors");
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingCommand()))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Throw(std::runtime_error("blocked by obstacle")));

    const std::unique_ptr<MissionControlImpl> mission_control =
        makeMissionControl(10, dir / "out.npy", /*verbose=*/true);
    (void)mission_control->runMission();

    const std::string contents = readAllFilesIn(dir);
    EXPECT_NE(contents.find("blocked by obstacle"), std::string::npos);
    // Only the errored step (step 2) is logged -- the plain working step (step 1) is not.
    EXPECT_NE(contents.find("step 2"), std::string::npos);
    EXPECT_EQ(contents.find("step 1"), std::string::npos);
}

TEST_F(MissionControl, VerboseFlagLogsACheckpointEvery500StepsInsteadOfEveryStep) {
    const std::filesystem::path dir = freshOutputDir("verbose_checkpoint");
    constexpr std::size_t kWorkingSteps = 500;
    std::size_t call_count = 0;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .Times(kWorkingSteps + 1)
        .WillRepeatedly(Invoke([&call_count](const DroneState&, const LidarScanResult*) {
            ++call_count;
            return call_count <= kWorkingSteps ? workingCommand() : finishedCommand();
        }));

    const std::unique_ptr<MissionControlImpl> mission_control =
        makeMissionControl(kWorkingSteps + 10, dir / "out.npy", /*verbose=*/true);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.steps, kWorkingSteps + 1);
    const std::string contents = readAllFilesIn(dir);
    EXPECT_NE(contents.find("checkpoint: steps=500"), std::string::npos);
    // Exactly one checkpoint should appear across a 501-step run (only step 500 is a multiple of
    // the checkpoint interval) -- not one line per step.
    EXPECT_EQ(contents.find("checkpoint: steps=500"), contents.rfind("checkpoint: steps="));
}

TEST_F(MissionControl, VerboseFlagDoesNotChangeTheMissionResult) {
    const std::filesystem::path dir = freshOutputDir("verbose_result_parity");
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()))
        .WillOnce(Return(scanCommand()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Throw(std::runtime_error("blocked by obstacle")))
        .WillOnce(Throw(std::runtime_error("blocked by obstacle")));

    const std::unique_ptr<MissionControlImpl> quiet_mission_control =
        makeMissionControl(10, dir / "quiet.npy", /*verbose=*/false);
    const MissionRunResult quiet_result = quiet_mission_control->runMission();

    const std::unique_ptr<MissionControlImpl> verbose_mission_control =
        makeMissionControl(10, dir / "verbose.npy", /*verbose=*/true);
    const MissionRunResult verbose_result = verbose_mission_control->runMission();

    EXPECT_EQ(quiet_result.status, verbose_result.status);
    EXPECT_EQ(quiet_result.steps, verbose_result.steps);
    ASSERT_EQ(quiet_result.errors.size(), verbose_result.errors.size());
    for (std::size_t i = 0; i < quiet_result.errors.size(); ++i) {
        EXPECT_EQ(quiet_result.errors[i].code, verbose_result.errors[i].code);
        EXPECT_EQ(quiet_result.errors[i].message, verbose_result.errors[i].message);
    }
}

TEST_F(MissionControl, OutputMapSaveFailureIsAppendedAsErrorWithoutErasingMissionResult) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(_)).WillOnce([]() { throw std::runtime_error("disk full"); });

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    MissionRunResult result;
    EXPECT_NO_THROW(result = mission_control->runMission());

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "OUTPUT_MAP_SAVE_FAILED");
    EXPECT_EQ(result.errors[0].message, "disk full");
}
