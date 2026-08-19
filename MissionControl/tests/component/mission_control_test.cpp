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
#include <memory>

using namespace common;
using namespace common::types;
using namespace MissionControl_322889890_315113738;
using ::testing::_;
using ::testing::Eq;
using ::testing::NiceMock;
using ::testing::Return;

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

// A movement command whose mocked IDroneMovement::advance() call (set up by the caller)
// fails, driving DroneControlImpl::step() to DroneStepStatus::Error with that message --
// the only way to reach Error now that IDroneControl is no longer injected directly.
MappingStepCommand advanceCommand() {
    MappingStepCommand command;
    MovementCommand movement;
    movement.type = MovementCommandType::Advance;
    movement.distance = 10.0 * isq::length[cm];
    command.movement = movement;
    return command;
}

class MissionControl : public ::testing::Test {
protected:
    NiceMock<test::GMockILidar> lidar_;
    NiceMock<test::GMockIGPS> gps_;
    NiceMock<test::GMockIDroneMovement> movement_;
    NiceMock<test::GMockIMutableMap3D> output_map_;
    NiceMock<test::GMockIMappingAlgorithm> algorithm_{
        MappingAlgorithmDependencies{missionConfig(1), lidarConfig(), droneConfig(), output_map_}};

    std::unique_ptr<MissionControlImpl> makeMissionControl(std::size_t max_steps,
                                                           const std::filesystem::path& output_map_file = "out.npy") {
        return std::make_unique<MissionControlImpl>(MissionControlDependencies{
            missionConfig(max_steps), droneConfig(), lidar_, gps_, movement_, output_map_, algorithm_,
            output_map_file});
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

TEST_F(MissionControl, ErrorStatusStopsLoopAndReportsErrorWithDetails) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(advanceCommand()));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(MovementResult{false, "blocked by obstacle"}));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Error);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "blocked by obstacle");
}

TEST_F(MissionControl, OutputMapSavedExactlyOnceWhenCompleted) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(_)).Times(1);

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    (void)mission_control->runMission();
}

TEST_F(MissionControl, OutputMapSavedExactlyOnceWhenError) {
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(advanceCommand()));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(MovementResult{false, "blocked by obstacle"}));
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
    // ErrorStatusStopsLoopAndReportsErrorWithDetails checks only the error message; the
    // "DRONE_CONTROL_ERROR" code and the steps count (++steps happens before the status check, so
    // an immediate Error must still report steps == 1, not 0) are separate output-contract details
    // that a bug could get wrong independently of the message.
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(advanceCommand()));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(MovementResult{false, "blocked by obstacle"}));

    const std::unique_ptr<MissionControlImpl> mission_control = makeMissionControl(10);
    const MissionRunResult result = mission_control->runMission();

    EXPECT_EQ(result.steps, 1u);
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
