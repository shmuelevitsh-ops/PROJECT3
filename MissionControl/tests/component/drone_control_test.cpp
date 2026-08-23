// Migrated from Project 2 (FILES PROJECT 2/tests/components/drone_control_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// no other behavioral changes from the Project 2 original. The DroneControlImpl DI
// seam (ILidar/IGPS/IDroneMovement/IMutableMap3D/IMappingAlgorithm, all by-reference)
// is unchanged from Project 2, so this file only needed the standard namespace/include
// port (§7.2) -- it is not one of the structurally-adapted MissionControl suites.

#include <MissionControl/DroneControlImpl.h>

#include "mocks/GMockIDroneMovement.h"
#include "mocks/GMockIGPS.h"
#include "mocks/GMockILidar.h"
#include "mocks/GMockIMappingAlgorithm.h"
#include "mocks/GMockIMutableMap3D.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace common;
using namespace common::types;
using namespace MissionControl_322889890_315113738;
using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::IsNull;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::Truly;

namespace {

MATCHER_P(OrientationEq, expected, "") {
    return arg.horizontal == expected.horizontal && arg.altitude == expected.altitude;
}

DroneConfigData droneConfig() {
    DroneConfigData config;
    config.radius = 5.0 * isq::length[cm];
    config.max_rotate = 45.0 * horizontal_angle[deg];
    config.max_advance = 50.0 * isq::length[cm];
    config.max_elevate = 40.0 * isq::length[cm];
    return config;
}

// Same as droneConfig(), but with limits far larger than any distance/angle used in this file, so
// oversized-command splitting (Optional Common-Issues row 8) never triggers. Used by tests whose
// point is something else entirely (OOB shortening magnitude, scan-position correctness) and
// that deliberately use a movement bigger than droneConfig()'s ordinary limits for their own,
// unrelated reasons -- constructing a local DroneControlImpl with this config (instead of using
// the fixture's control_) isolates them from splitting without changing their original numbers.
DroneConfigData unsplittableDroneConfig() {
    DroneConfigData config = droneConfig();
    config.max_rotate = 1.0e9 * horizontal_angle[deg];
    config.max_advance = 1.0e9 * isq::length[cm];
    config.max_elevate = 1.0e9 * isq::length[cm];
    return config;
}

MissionConfigData missionConfig() {
    MissionConfigData config;
    config.max_steps = 100;
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

MovementCommand rotateCommand(HorizontalAngle angle, RotationDirection direction = RotationDirection::Left) {
    MovementCommand command;
    command.type = MovementCommandType::Rotate;
    command.rotation = direction;
    command.angle = angle;
    return command;
}

MovementCommand advanceCommand(PhysicalLength distance) {
    MovementCommand command;
    command.type = MovementCommandType::Advance;
    command.distance = distance;
    return command;
}

MovementCommand elevateCommand(PhysicalLength distance) {
    MovementCommand command;
    command.type = MovementCommandType::Elevate;
    command.distance = distance;
    return command;
}

MovementCommand hoverCommand() {
    MovementCommand command;
    command.type = MovementCommandType::Hover;
    return command;
}

// A single-hit scan result, used throughout as the generic "the scan succeeded" filler wherever
// a test's point is something else entirely (movement/scan ordering, orientation forwarding,
// chunk sequencing, GPS baseline handling, ...). An empty vector no longer serves this purpose:
// since Optional Common-Issues row 6 ("The LiDAR returns an empty vector"), an empty
// lidar_.scan() result is retried and eventually thrown on, so any filler scan return must be
// non-empty to avoid accidentally exercising that retry logic.
LidarScanResult nonEmptyScanResult() { return LidarScanResult{LidarHit{}}; }

// Returns a gmock action body that replies with successive entries of `positions` on successive
// calls, holding at the last entry for any calls beyond the list's length. Used throughout to
// give gps_.position() a sequence of readings consistent with DroneControlImpl's own internal
// position tracking (Optional Common-Issues row 12: every successful movement chunk is now
// followed by a post-movement GPS re-read that must match the physically-expected position, in
// addition to the pre-existing pre-step read consumed as Algorithm state / row-11 validation) --
// instead of a single canned value that a real, moving drone would never actually report twice
// in a row. A shared_ptr index (rather than a mutable lambda) keeps every copy gmock makes of the
// action referring to the same cursor.
[[nodiscard]] auto gpsPositionSequence(std::vector<Position3D> positions) {
    auto index = std::make_shared<std::size_t>(0);
    return [positions = std::move(positions), index](){
        const Position3D pos = positions[std::min(*index, positions.size() - 1)];
        ++*index;
        return pos;
    };
}

// Generous, effectively unbounded map used as the fixture's default MapConfig, so that tests
// unrelated to out-of-bounds handling (which don't stub output_map_.getMapConfig() themselves)
// keep exercising the full, unshortened Advance/Elevate path exactly as before that handling was
// added. Tests that specifically exercise out-of-bounds handling override this via their own
// EXPECT_CALL/ON_CALL on output_map_.
MapConfig unboundedMapConfig() {
    MapConfig config;
    config.boundaries = MappingBounds{
        -100000.0 * x_extent[cm], 100000.0 * x_extent[cm], -100000.0 * y_extent[cm],
        100000.0 * y_extent[cm], -100000.0 * z_extent[cm], 100000.0 * z_extent[cm]};
    return config;
}

MapConfig mapConfigWithBounds(MappingBounds bounds) {
    MapConfig config;
    config.boundaries = bounds;
    return config;
}

// A real (half-open, min inclusive / max exclusive) bounds check, matching the actual Map3DImpl
// semantics DroneControlImpl must not hard-code -- used only as the *test double's* isInBounds()
// behavior, so DroneControlImpl is exercised exactly as it would be against a real map, via the
// published IMap3D interface alone. Captures `bounds` by value so it never needs to call back into
// output_map_.getMapConfig() (which would silently add extra calls a test wasn't expecting).
[[nodiscard]] auto isInBoundsFor(const MappingBounds& bounds) {
    return [bounds](const Position3D& pos) {
        return pos.x >= bounds.min_x && pos.x < bounds.max_x && pos.y >= bounds.min_y &&
               pos.y < bounds.max_y && pos.z >= bounds.min_height && pos.z < bounds.max_height;
    };
}

class DroneControl : public ::testing::Test {
protected:
    NiceMock<test::GMockILidar> lidar_;
    NiceMock<test::GMockIGPS> gps_;
    NiceMock<test::GMockIDroneMovement> movement_;
    NiceMock<test::GMockIMutableMap3D> output_map_;
    NiceMock<test::GMockIMappingAlgorithm> algorithm_{
        MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), output_map_}};
    DroneControlImpl control_{
        droneConfig(), missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    void SetUp() override {
        ON_CALL(output_map_, getMapConfig()).WillByDefault(Return(unboundedMapConfig()));
        ON_CALL(output_map_, isInBounds(_)).WillByDefault(Invoke(isInBoundsFor(unboundedMapConfig().boundaries)));
        // Default to a non-empty scan (Optional Common-Issues row 6 treats an empty vector as a
        // faulty LiDAR result to retry/throw on) so tests that don't care about scan content --
        // and so never override this -- don't accidentally exercise that retry logic.
        ON_CALL(lidar_, scan(_)).WillByDefault(Return(nonEmptyScanResult()));
    }
};

} // namespace

TEST_F(DroneControl, FirstStepPassesNullLatestScanToAlgorithm) {
    // A Hover movement (rather than a bare default-constructed command) keeps this a legitimate
    // "nothing happens, Working" step rather than a faulty NOOP (Optional Common-Issues row 4).
    MappingStepCommand command;
    command.movement = hoverCommand();
    EXPECT_CALL(algorithm_, nextStep(_, IsNull())).WillOnce(Return(command));

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, RotateMovementCallsRotateBeforeScan) {
    MappingStepCommand command;
    command.movement = rotateCommand(10.0 * horizontal_angle[deg], RotationDirection::Left);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 10.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
}

TEST_F(DroneControl, AdvanceMovementCallsAdvanceBeforeScan) {
    // Post-movement GPS (Optional Common-Issues row 12) must agree with a 20cm advance along the
    // default (0deg) heading, i.e. x=20cm, or the step would report Error instead of dispatching
    // the scan this test is actually about.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{20.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(20.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(20.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
}

TEST_F(DroneControl, ElevateMovementCallsElevateBeforeScan) {
    // Post-movement GPS (row 12) must agree with a 15cm elevate, i.e. z=15cm.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 15.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = elevateCommand(15.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, elevate(15.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
}

// New — closes a Project 2 mutation-coverage gap (DRO08, see
// PROJ2_TESTS_PLAN.md §8.1/§9). One-line variant of
// ElevateMovementCallsElevateBeforeScan above with a negative distance:
// nothing previously asserted that a commanded *descent* (distance < 0)
// reaches IDroneMovement::elevate() unchanged. DRO08's mutation silently
// clamps any negative elevate distance to 0 before it reaches movement_,
// turning a commanded descent into a no-op the algorithm believes succeeded.
TEST_F(DroneControl, ElevateMovementForwardsNegativeDistanceUnchanged) {
    // Post-movement GPS (row 12) must agree with a -10cm elevate, i.e. z=-10cm.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], -10.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = elevateCommand(-10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, elevate(-10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
}

TEST_F(DroneControl, HoverMovementDoesNotCallAnyMovementMethod) {
    MappingStepCommand command;
    command.movement = hoverCommand();
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, NoScanOrientationDoesNotCallLidarScan) {
    // scan_orientation unset; movement set to Hover so this stays a legitimate "nothing happens"
    // step rather than a faulty NOOP (Optional Common-Issues row 4).
    MappingStepCommand command;
    command.movement = hoverCommand();
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(_)).Times(0);

    (void)control_.step();
}

TEST_F(DroneControl, ScanOrientationSetCallsLidarScanWithGivenOrientation) {
    const Orientation orientation{30.0 * horizontal_angle[deg], 5.0 * altitude_angle[deg]};
    MappingStepCommand command;
    command.scan_orientation = orientation;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(OrientationEq(orientation))).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
}

TEST_F(DroneControl, ScanResultIsPassedAsLatestScanToNextStepOnly) {
    LidarHit hit;
    hit.distance = 42.0 * isq::length[cm];
    const LidarScanResult scan_result{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    // Hover (not a bare no-movement/no-scan command) so these idle steps stay legitimate
    // "nothing happens" results rather than faulty NOOPs (Optional Common-Issues row 4).
    MappingStepCommand idle_command;
    idle_command.movement = hoverCommand();

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isSingleHitScan = [](const LidarScanResult* scan) { return scan != nullptr && scan->size() == 1; };

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(scan_result));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(idle_command));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(idle_command));

    (void)control_.step(); // step 1: scans, stores latest_scan_
    (void)control_.step(); // step 2: receives non-null latest_scan_, does not scan
    (void)control_.step(); // step 3: receives nullptr again, since step 2 had no scan
}

TEST_F(DroneControl, WorkingStatusMapsToContinue) {
    // Hover movement so this is a legitimate Working "nothing happens" step, not a faulty NOOP
    // (Optional Common-Issues row 4).
    MappingStepCommand command;
    command.status = AlgorithmStatus::Working;
    command.movement = hoverCommand();
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, FinishedStatusMapsToCompletedWithFinishedMessage) {
    MappingStepCommand command;
    command.status = AlgorithmStatus::Finished;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Completed);
    EXPECT_EQ(result.message, "mapping finished");
}

TEST_F(DroneControl, FinishedWithUnmappableVoxelsMapsToCompletedWithDistinctMessage) {
    MappingStepCommand command;
    command.status = AlgorithmStatus::FinishedWithUnmappableVoxels;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Completed);
    EXPECT_EQ(result.message, "mapping finished with unmappable voxels remaining");
}

TEST_F(DroneControl, UnrecognizedAlgorithmStatusMapsToErrorNotSilentContinue) {
    // AlgorithmStatus is a closed 3-value enum, but step()'s status-mapping switch has a
    // defensive fallback (no `default:` case; an explicit return after the switch) for any value
    // that doesn't match a known enumerator -- reachable only via an out-of-range value such as
    // this one, which is well-defined to construct for a scoped enum backed by `int`. A bug that
    // removes this fallback (defaulting silently to Continue) would let an unrecognized status
    // slip through as if mapping were still in progress, instead of surfacing the problem.
    MappingStepCommand command;
    command.status = static_cast<AlgorithmStatus>(999);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Error)
        << "an unrecognized AlgorithmStatus value must be reported as an error, not silently "
           "treated as Continue";
    EXPECT_FALSE(result.message.empty()) << "the error must carry a descriptive message";
}

// Optional Common-Issues row 7 ("The movement driver returns false"): a single false no longer
// terminates the step immediately -- it retries the same chunk up to kMaxMovementAttempts total
// driver calls (see the dedicated row-7 test section below). Only once all of them return false
// does step() give up -- and, per row 7, that is a throw, not an Error DroneStepResult.
TEST_F(DroneControl, MovementFailureRetriesExhaustedThrowsWithoutScanningOrAdvancingStepIndex) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{}; // should never be reached
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .Times(3)
        .WillRepeatedly(Return(MovementResult{false, "blocked by obstacle"}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u);
}

// Movement collision scenario: MockMovement (the real implementation, not this mock) rejects a
// real wall collision by throwing rather than returning success=false, since it alone can see
// the hidden map. step() must catch that narrowly around the movement dispatch, report it as an
// Error DroneStepResult without scanning or advancing step_index_, and must never let the
// exception itself escape step() -- and, unlike a returned success=false (row 7), it is never
// retried.
TEST_F(DroneControl, MovementExceptionReturnsErrorWithoutScanningOrAdvancingStepIndex) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{}; // should never be reached
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .WillOnce([]() -> MovementResult { throw std::runtime_error("drone collided with a wall"); });
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    DroneStepResult result;
    EXPECT_NO_THROW(result = control_.step());

    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(result.message, "drone collided with a wall");
    EXPECT_EQ(control_.state().step_index, 0u);
}

TEST_F(DroneControl, StepIndexIncrementsOnSuccessfulStepsOnly) {
    // Hover, no scan, Working: a legitimate "nothing happens" step, not a faulty NOOP (Optional
    // Common-Issues row 4).
    MappingStepCommand command;
    command.movement = hoverCommand();
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(3).WillRepeatedly(Return(command));

    ASSERT_EQ(control_.state().step_index, 0u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 1u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 2u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 3u);
}

// New — closes a Project 2 mutation-coverage gap (DRO09, see
// PROJ2_TESTS_PLAN.md §8.2). Captures the DroneState argument the algorithm
// actually receives on each call, rather than reading it back afterward via
// control_.state() (as StepIndexIncrementsOnSuccessfulStepsOnly above does) --
// a more direct check of what nextStep() itself observes.
TEST_F(DroneControl, StepIndexSeenByAlgorithmIncrementsAcrossFirstTwoSteps) {
    // Hover, no scan, Working: a legitimate "nothing happens" step, not a faulty NOOP (Optional
    // Common-Issues row 4).
    MappingStepCommand command;
    command.movement = hoverCommand();
    DroneState first_seen_state{};
    DroneState second_seen_state{};
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(DoAll(SaveArg<0>(&first_seen_state), Return(command)))
        .WillOnce(DoAll(SaveArg<0>(&second_seen_state), Return(command)));

    (void)control_.step();
    (void)control_.step();

    EXPECT_EQ(first_seen_state.step_index, 0u)
        << "the algorithm must see step_index 0 on the very first step() call";
    EXPECT_EQ(second_seen_state.step_index, 1u)
        << "the algorithm must see step_index 1 on the second step() call";
}

TEST_F(DroneControl, StepIndexIsNotIncrementedIfScanningThrows) {
    // ++step_index_ must happen only after the scan block has fully completed, not before it: if
    // lidar_.scan() throws mid-step, step_index_ must still reflect the *last actually completed*
    // step, not a step that never finished. A bug that moves the increment earlier (e.g. right
    // after movement, before scanning) would leave step_index_ one ahead of what was truly
    // scanned whenever an exception escapes from the scan block. A scan/map-processing exception
    // (Phase 2B fix) is caught locally and reported as DroneStepStatus::Error -- it no longer
    // escapes step() as a raw exception.
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce([]() -> LidarScanResult { throw std::runtime_error("lidar fault"); });

    ASSERT_EQ(control_.state().step_index, 0u);
    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(result.message, "lidar fault");
    EXPECT_EQ(control_.state().step_index, 0u)
        << "step_index_ must not be incremented for a step whose scan never completed";
}

TEST_F(DroneControl, PositionAndHeadingAreReReadAfterMovementBeforeScanning) {
    // ScanResultToVoxels::applyToMap() must see the drone's *post-movement* position/heading, not
    // whatever was read at the top of step() before movement ran. A bug that reuses the initial
    // `state` instead of re-querying gps_ after movement would call position()/heading() only
    // once each here instead of twice.
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    // Second reading must equal the physically-expected post-movement position (x=10cm along the
    // default 0deg heading) or row 12's validation would keep retrying instead of proceeding
    // straight to the scan this test's Times(2) is meant to isolate.
    EXPECT_CALL(gps_, position())
        .Times(2)
        .WillOnce(Return(Position3D{}))
        .WillOnce(Return(Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
    EXPECT_CALL(gps_, heading()).Times(2).WillRepeatedly(Return(Orientation{}));

    (void)control_.step();
}

TEST_F(DroneControl, ScanResultAppliedAtPostMovementPositionNotPreMovement) {
    // A narrower variant of the bug above that targets only the *value* passed as the scan
    // origin into ScanResultToVoxels::applyToMap, not whether gps_.position() gets called again
    // (a bug could still call gps_.position() twice -- satisfying the call-count check above --
    // while wrongly discarding the second result and passing the first, pre-movement, value into
    // applyToMap instead). Verified by checking *where* a resulting Occupied voxel actually lands.
    // Uses a local DroneControlImpl with unsplittableDroneConfig(): the 100cm advance below is
    // deliberately larger than droneConfig()'s ordinary 50cm max_advance for unrelated reasons
    // (a clean, easy-to-read distance), and this test has nothing to do with oversized-command
    // splitting -- it must not become one.
    DroneControlImpl unsplit_control{
        unsplittableDroneConfig(), missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    MappingStepCommand command;
    command.movement = advanceCommand(100.0 * isq::length[cm]);
    command.scan_orientation = Orientation{}; // straight ahead, no offset from heading
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(movement_, advance(100.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const Position3D pre_move_pos{};                              // x=0cm
    const Position3D post_move_pos{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    {
        InSequence seq;
        EXPECT_CALL(gps_, position()).WillOnce(Return(pre_move_pos));
        EXPECT_CALL(gps_, position()).WillOnce(Return(post_move_pos));
    }
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    LidarHit hit;
    hit.distance = 30.0 * isq::length[cm];
    hit.angle = Orientation{};
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{hit}));
    EXPECT_CALL(lidar_, config()).WillRepeatedly(Return(lidarConfig()));

    MapConfig map_config{
        MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm], -1000.0 * y_extent[cm],
                      1000.0 * y_extent[cm], -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]},
        Position3D{}, 10.0 * isq::length[cm]};
    EXPECT_CALL(output_map_, getMapConfig()).WillRepeatedly(Return(map_config));
    EXPECT_CALL(output_map_, isInBounds(_)).WillRepeatedly(Return(true));
    EXPECT_CALL(output_map_, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Unmapped));

    // Hit position if the scan origin is correctly the post-movement position (100cm + 30cm =
    // 130cm) vs. if it wrongly used the stale pre-movement position (0cm + 30cm = 30cm).
    auto isNearX = [](double expected_cm) {
        return [expected_cm](const Position3D& p) {
            return std::abs(p.x.force_numerical_value_in(cm) - expected_cm) < 1.0;
        };
    };
    // gmock matches the most-recently-declared expectation first, so the catch-all must come
    // before the specific assertions below for those to take priority.
    EXPECT_CALL(output_map_, set(_, _)).Times(testing::AnyNumber());
    EXPECT_CALL(output_map_, set(Truly(isNearX(130.0)), VoxelOccupancy::Occupied)).Times(1);
    EXPECT_CALL(output_map_, set(Truly(isNearX(30.0)), VoxelOccupancy::Occupied)).Times(0);

    (void)unsplit_control.step();
}

TEST_F(DroneControl, MovementFailureDoesNotClearPreviousLatestScan) {
    // Documented in DroneControlImpl::step(): on exhausted movement retries (Optional
    // Common-Issues row 7), latest_scan_ is left untouched. A bug that unconditionally resets it
    // to nullopt (matching the no-scan-this-step branch) would lose the previous step's scan data
    // instead of preserving it.
    LidarHit hit;
    hit.distance = 10.0 * isq::length[cm];
    const LidarScanResult scan_result{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    MappingStepCommand failing_move_command;
    failing_move_command.movement = advanceCommand(10.0 * isq::length[cm]);

    // Hover so this idle step stays legitimate rather than a faulty NOOP (Optional Common-Issues
    // row 4).
    MappingStepCommand idle_command;
    idle_command.movement = hoverCommand();

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isSingleHitScan = [](const LidarScanResult* scan) { return scan != nullptr && scan->size() == 1; };

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(scan_result));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(failing_move_command));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .Times(3)
        .WillRepeatedly(Return(MovementResult{false, "blocked"}));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(idle_command));

    (void)control_.step();  // step 1: scans, stores latest_scan_ (1 hit)
    EXPECT_THROW((void)control_.step(), std::runtime_error)
        << "step 2: 3 exhausted movement retries throw before ever touching latest_scan_";
    (void)control_.step();  // step 3: must still see step 1's latest_scan_, not nullptr
}

TEST_F(DroneControl, StateReturnsGpsPositionAndHeading) {
    const Position3D position{5.0 * x_extent[cm], 6.0 * y_extent[cm], 7.0 * z_extent[cm]};
    const Orientation heading{20.0 * horizontal_angle[deg], 3.0 * altitude_angle[deg]};
    EXPECT_CALL(gps_, position()).WillRepeatedly(Return(position));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(heading));

    const DroneState state = control_.state();

    EXPECT_EQ(state.position.x, position.x);
    EXPECT_EQ(state.position.y, position.y);
    EXPECT_EQ(state.position.z, position.z);
    EXPECT_EQ(state.heading.horizontal, heading.horizontal);
    EXPECT_EQ(state.heading.altitude, heading.altitude);
}

// --- Optional Common-Issues scenario: out-of-bounds Advance/Elevate handling ---------------
//
// Staff table combines two rows describing the same situation with different handling
// strategies ("ignore the illegal movement" / "amend the movement"): DroneControlImpl applies
// whichever applies -- shorten to the maximum legal non-zero distance in the requested
// direction when one exists, otherwise drop the movement entirely without calling
// IDroneMovement. output_map_.isInBounds() is the sole authority on legality;
// output_map_.getMapConfig().boundaries is used only to estimate how much distance should fit.
// Every test below stubs isInBounds() via isInBoundsFor(bounds) -- a real half-open bounds check
// -- rather than a canned true/false, so DroneControlImpl is exercised exactly as it would be
// against a real map. Distances below use a clear, generous margin from the boundary (tens of cm)
// so assertions on the shortened distance don't depend on exact floating-point equality at the
// boundary.

namespace {

// True if `distance` (a PhysicalLength) is within `tolerance_cm` of `expected_cm`, without
// relying on exact equality -- the amended distance at an upper bound is nudged minimally inside
// it (via std::nextafter), not to some fixed offset from it.
auto isNearCm(double expected_cm, double tolerance_cm = 0.01) {
    return [expected_cm, tolerance_cm](PhysicalLength distance) {
        const double actual_cm = distance.force_numerical_value_in(cm);
        return actual_cm > expected_cm - tolerance_cm && actual_cm < expected_cm + tolerance_cm;
    };
}

// Stubs both output_map_'s getMapConfig() and isInBounds() consistently with `bounds`, so
// DroneControlImpl's shortening estimate and its final isInBounds() legality check always agree
// on the same map.
void stubMapBounds(NiceMock<test::GMockIMutableMap3D>& output_map, MappingBounds bounds) {
    ON_CALL(output_map, getMapConfig()).WillByDefault(Return(mapConfigWithBounds(bounds)));
    ON_CALL(output_map, isInBounds(_)).WillByDefault(Invoke(isInBoundsFor(bounds)));
}

// Returns a gmock action body for movement_.advance()/elevate() that records the exact distance
// it was actually invoked with (in cm) into `captured_distance_cm` and reports success. Used by
// the boundary-shortening tests below so their post-movement GPS stub can report the true legal
// position the implementation itself dispatched -- which, since output_map_.isInBounds() is the
// sole authority on legality, may be nudged (via std::nextafter) a hair inside an exclusive upper
// bound rather than sitting exactly on the boundary literal (which row 12 now correctly rejects
// as out of bounds, no matter how numerically close it is).
[[nodiscard]] auto captureDistanceAndSucceed(std::shared_ptr<double> captured_distance_cm) {
    return [captured_distance_cm](PhysicalLength distance) {
        *captured_distance_cm = distance.force_numerical_value_in(cm);
        return MovementResult{true, ""};
    };
}

} // namespace

TEST_F(DroneControl, AdvanceWithinBoundsIsForwardedUnchanged) {
    // Post-movement GPS (row 12) must agree with a 20cm advance along the default heading.
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{20.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(20.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(20.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AdvancePartiallyCrossingBoundIsShortenedAndDispatched) {
    // Current x=0, heading 0deg (advancing in +x), upper x bound at 30cm, requested distance
    // 50cm: only the first ~30cm is legal, and (the upper bound being exclusive) the dispatched
    // distance is nudged a hair *below* 30cm. Post-movement GPS (row 12) must report that actual
    // dispatched -- and therefore genuinely in-bounds -- position, not the boundary literal
    // itself, which row 12 would now reject as out of bounds regardless of numerical closeness.
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 30.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    auto dispatched_distance_cm = std::make_shared<double>(0.0);
    EXPECT_CALL(movement_, advance(Truly(isNearCm(30.0))))
        .WillOnce(Invoke(captureDistanceAndSucceed(dispatched_distance_cm)));
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))
        .WillOnce(Invoke([dispatched_distance_cm]() {
            return Position3D{
                *dispatched_distance_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
        }));

    MappingStepCommand command;
    command.movement = advanceCommand(50.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AdvanceWithNoLegalDistanceRemainingIsIgnored) {
    // Drone sitting exactly at the *inclusive* lower x boundary (a valid in-bounds state, unlike
    // sitting exactly at an upper/exclusive one) and heading 180deg (advancing further in -x):
    // any outward distance at all would leave the map, so nothing is legal to dispatch.
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Return(Position3D{-30.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{180.0 * horizontal_angle[deg], {}}));
    stubMapBounds(output_map_, MappingBounds{-30.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(50.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(_)).Times(0);

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ElevateCrossingUpperBoundIsShortened) {
    // Current z=0, upper height bound at 50cm, requested elevate distance 80cm. The resulting
    // ~50cm legal distance exceeds droneConfig()'s ordinary 40cm max_elevate, which is unrelated
    // to what this test means to exercise (pure OOB shortening magnitude, not row-8 splitting) --
    // uses a local DroneControlImpl with unsplittableDroneConfig() to isolate that.
    DroneControlImpl unsplit_control{
        unsplittableDroneConfig(), missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    // Post-movement GPS (row 12) must report the actual dispatched -- nudged a hair below the
    // exclusive 50cm upper bound, and therefore genuinely in-bounds -- position, not the boundary
    // literal itself.
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 50.0 * z_extent[cm]});

    auto dispatched_distance_cm = std::make_shared<double>(0.0);
    EXPECT_CALL(movement_, elevate(Truly(isNearCm(50.0))))
        .WillOnce(Invoke(captureDistanceAndSucceed(dispatched_distance_cm)));
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))
        .WillOnce(Invoke([dispatched_distance_cm]() {
            return Position3D{
                0.0 * x_extent[cm], 0.0 * y_extent[cm], *dispatched_distance_cm * z_extent[cm]};
        }));

    MappingStepCommand command;
    command.movement = elevateCommand(80.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = unsplit_control.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, NegativeElevateCrossingLowerBoundIsShortened) {
    // Current z=50, lower height bound at 0 (inclusive), requested elevate distance -80cm
    // (descending): the legal descent stops exactly at 0, so the shortened distance is exactly
    // -50cm -- the lower bound is inclusive, so no inward nudge is needed there. Same
    // unsplittableDroneConfig() isolation as ElevateCrossingUpperBoundIsShortened above (50cm
    // legal distance exceeds the ordinary 40cm max_elevate).
    DroneControlImpl unsplit_control{
        unsplittableDroneConfig(), missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    // Post-movement GPS (row 12) must agree with the descent landing exactly at z=0.
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Invoke(gpsPositionSequence(
            {Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]},
             Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              0.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = elevateCommand(-80.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, elevate(Truly(isNearCm(-50.0, 1e-6))))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = unsplit_control.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, IgnoredMovementDoesNotPreventRequestedScanFromRunning) {
    // Same illegal-advance setup as AdvanceWithNoLegalDistanceRemainingIsIgnored, but this step
    // also requests a scan: dropping the movement must not stop the rest of the step.
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Return(Position3D{-30.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{180.0 * horizontal_angle[deg], {}}));
    stubMapBounds(output_map_, MappingBounds{-30.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(50.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AdvanceParallelToUpperBoundIsNotShortenedByHeadingFloatingPointResidue) {
    // Regression test: heading 90deg is meant to advance purely in +y, but si::cos(90deg) is a
    // tiny nonzero double (order 1e-17), not an exact zero, due to floating-point rounding of the
    // deg->rad conversion. That residue is a fixed fraction of distance, not of the gap to a
    // boundary, so a large enough requested distance turns it into a non-negligible *absolute* x
    // displacement: at distance=5e7cm it is of order 1e-9cm -- enough to cross an x gap of exactly
    // 1e-9cm if it were ever treated as a real x-component. Without the negligible-
    // direction-component fix, that phantom x-component -- x is otherwise legal and barely a
    // hair from its own upper bound -- would incorrectly shorten or drop a movement that is
    // entirely legal in the direction actually requested (pure +y, well inside its own bound).
    // The movement must be dispatched unchanged.
    //
    // Uses a local DroneControlImpl with unsplittableDroneConfig(): 5e7cm is deliberately far
    // beyond droneConfig()'s ordinary 50cm max_advance purely to make the floating-point residue
    // above numerically significant -- an unrelated concern from oversized-command splitting
    // (row 8). Without this isolation, this test would silently become a row-8 splitting test
    // instead of the OOB floating-point regression it is meant to be.
    DroneControlImpl unsplit_control{
        unsplittableDroneConfig(), missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    const auto x_near_bound = 30.0 * x_extent[cm] - 1.0e-9 * x_extent[cm];
    // Post-movement GPS (row 12) must agree with a pure +y advance of 5e7cm: x unchanged (the
    // same negligible-direction-component zeroing this test is about applies equally to
    // DroneControlImpl's own expected-position math), y advances by the full 5e7cm.
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Invoke(gpsPositionSequence(
            {Position3D{x_near_bound, 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{x_near_bound, 5.0e7 * y_extent[cm], 0.0 * z_extent[cm]}})));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{90.0 * horizontal_angle[deg], {}}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 30.0 * x_extent[cm],
                                              -1.0e12 * y_extent[cm], 1.0e12 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(5.0e7 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(5.0e7 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = unsplit_control.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, RotateAndHoverDoNotConsultMapBoundsOrGetShortened) {
    // Rotate/Hover are not position-changing: the row-10 OOB-amendment logic (which alone
    // consults getMapConfig() for its boundaries estimate) must never run for them, and their
    // dispatch must behave exactly as before that handling was added. output_map_.isInBounds()
    // itself is *not* asserted Times(0) here: every step() now also calls it once, unconditionally,
    // against the pre-step GPS reading for row-11 validation, regardless of movement type.
    EXPECT_CALL(output_map_, getMapConfig()).Times(0);

    MappingStepCommand rotate_step;
    rotate_step.movement = rotateCommand(10.0 * horizontal_angle[deg], RotationDirection::Right);
    MappingStepCommand hover_step;
    hover_step.movement = hoverCommand();

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(rotate_step));
    EXPECT_CALL(movement_, rotate(RotationDirection::Right, 10.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(hover_step));
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);

    (void)control_.step();
    (void)control_.step();
}

TEST_F(DroneControl, ShortenedAdvanceDistanceUsesNarrowerBoundFromGetMapConfig) {
    // Same setup as AdvancePartiallyCrossingBoundIsShortenedAndDispatched, but with a narrower
    // upper x bound (15cm instead of 30cm): the shortened distance dispatched must track
    // whatever output_map_.getMapConfig() reports, not a fixed constant. Post-movement GPS
    // (row 12) must report the actual dispatched -- nudged a hair below the exclusive bound, and
    // therefore genuinely in-bounds -- position, not the boundary literal itself.
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 15.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    auto dispatched_distance_cm = std::make_shared<double>(0.0);
    EXPECT_CALL(movement_, advance(Truly(isNearCm(15.0))))
        .WillOnce(Invoke(captureDistanceAndSucceed(dispatched_distance_cm)));
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))
        .WillOnce(Invoke([dispatched_distance_cm]() {
            return Position3D{
                *dispatched_distance_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
        }));

    MappingStepCommand command;
    command.movement = advanceCommand(50.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

// --- Optional Common-Issues row 8: oversized-command splitting -----------------------------
//
// "The Algorithm returned a movement bigger than the max allowed" -> DroneControlImpl splits it
// into 2+ commands, each as a separate step() call, without calling mapping_algorithm_.nextStep()
// again until every chunk of the original command has been dispatched. The existing OOB amendment
// (above) is applied first to establish the total *legal* movement; splitting only ever shortens
// that already-legal amount for size, and never re-checks map bounds between chunks.

TEST_F(DroneControl, AdvanceOversizedSplitsIntoThreeChunksWithoutRecallingAlgorithm) {
    // droneConfig()'s max_advance is 50cm; 150cm requested -> 50 + 50 + 50 across 3 step() calls,
    // fetching the command from the algorithm only once. Post-movement GPS (row 12) must agree
    // with the cumulative x after each chunk (50, 100, 150); the pre-step reading interleaved
    // before each (row 11) is otherwise unused once a sequence is already pending, so it is left
    // at the origin.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{150.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ElevateOversizedSplitsPreservingNegativeDirection) {
    // max_elevate is 40cm; a descent of -95cm -> -40, -40, -15 (sign preserved in every chunk).
    // Post-movement GPS (row 12) must agree with the cumulative z after each chunk.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], -40.0 * z_extent[cm]},
             Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], -80.0 * z_extent[cm]},
             Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], -95.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = elevateCommand(-95.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, elevate(-40.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, elevate(-40.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, elevate(-15.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, RotateOversizedSplitsPreservingDirection) {
    // max_rotate is 45deg; 100deg Left -> 45, 45, 10, each chunk still RotationDirection::Left.
    MappingStepCommand command;
    command.movement = rotateCommand(100.0 * horizontal_angle[deg], RotationDirection::Left);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 45.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 45.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 10.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ScanFromOriginalCommandIsAttachedOnlyToFinalChunk) {
    // The original command also requests a scan: it must not appear until after all 3 movement
    // chunks have been dispatched. InSequence enforces that ordering directly -- if scan were
    // wrongly attached to an earlier chunk, the sequence would be violated. Post-movement GPS
    // (row 12) must agree with the cumulative x after each chunk.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{150.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step();
    (void)control_.step();
    (void)control_.step();
}

TEST_F(DroneControl, FinishedStatusIsDeferredUntilFinalChunk) {
    // Intermediate chunks must report Continue regardless of the original command's status, so
    // MissionControl cannot see Finished (and stop the mission) before the whole legal movement
    // has actually executed. Post-movement GPS (row 12) must agree with the cumulative x after
    // each chunk.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{150.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    command.status = AlgorithmStatus::Finished;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));
    EXPECT_CALL(movement_, advance(_)).WillRepeatedly(Return(MovementResult{true, ""}));

    const DroneStepResult chunk1 = control_.step();
    const DroneStepResult chunk2 = control_.step();
    const DroneStepResult chunk3 = control_.step();

    EXPECT_EQ(chunk1.status, DroneStepStatus::Continue);
    EXPECT_EQ(chunk2.status, DroneStepStatus::Continue);
    EXPECT_EQ(chunk3.status, DroneStepStatus::Completed);
    EXPECT_EQ(chunk3.message, "mapping finished");
}

TEST_F(DroneControl, FinishedWithUnmappableVoxelsStatusIsDeferredUntilFinalChunk) {
    // Post-movement GPS (row 12) must agree with the cumulative x after each chunk.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{150.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    command.status = AlgorithmStatus::FinishedWithUnmappableVoxels;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));
    EXPECT_CALL(movement_, advance(_)).WillRepeatedly(Return(MovementResult{true, ""}));

    const DroneStepResult chunk1 = control_.step();
    const DroneStepResult chunk2 = control_.step();
    const DroneStepResult chunk3 = control_.step();

    EXPECT_EQ(chunk1.status, DroneStepStatus::Continue);
    EXPECT_EQ(chunk2.status, DroneStepStatus::Continue);
    EXPECT_EQ(chunk3.status, DroneStepStatus::Completed);
    EXPECT_EQ(chunk3.message, "mapping finished with unmappable voxels remaining");
}

TEST_F(DroneControl, StepIndexIncrementsOnceForEachSplitChunk) {
    // This test also calls control_.state() between step()s (each of which itself reads
    // gps_.position()), so a fixed pre-computed sequence of readings (as used elsewhere in this
    // file) would be thrown out of alignment by those extra calls. Instead, gps_.position()
    // always reports a `cumulative_x_cm` that movement_.advance()'s own action keeps up to date
    // -- correct for row-12 validation regardless of how many extra times either mock gets
    // called.
    auto cumulative_x_cm = std::make_shared<double>(0.0);
    ON_CALL(gps_, position()).WillByDefault(Invoke([cumulative_x_cm]() {
        return Position3D{*cumulative_x_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    }));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));
    EXPECT_CALL(movement_, advance(_)).WillRepeatedly(Invoke([cumulative_x_cm](PhysicalLength distance) {
        *cumulative_x_cm += distance.force_numerical_value_in(cm);
        return MovementResult{true, ""};
    }));

    ASSERT_EQ(control_.state().step_index, 0u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 1u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 2u);
    (void)control_.step();
    EXPECT_EQ(control_.state().step_index, 3u);
}

TEST_F(DroneControl, OutOfBoundsAmendmentAppliesBeforeSplitting) {
    // Requested 150cm, but only 80cm is legal inside mission bounds (an inclusive lower x bound
    // at -80, heading 180deg advancing in -x from x=0), and max_advance is 50cm: the legal 80cm
    // -- not the originally-requested 150cm -- is what gets split, into 50 + 30, not
    // 50 + 50 + (an ignored/illegal remainder). Post-movement GPS (row 12) must agree with the
    // cumulative -x after each chunk (-50, then -80).
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{-50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{-80.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{180.0 * horizontal_angle[deg], {}}));
    stubMapBounds(output_map_, MappingBounds{-80.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(30.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, MovementFailureOnIntermediateChunkPreservesFailureSemantics) {
    // Chunk 1 of 3 (50cm) succeeds; chunk 2 fails 3 times in a row (Optional Common-Issues row
    // 7's exhausted retry budget, all against the same 50cm chunk -- never chunk 3's would-be
    // 50cm). Exhausting it throws rather than returning Error: no scan, step_index_ not
    // incremented for it. Then: a further step() call must fetch a brand-new command from the
    // algorithm rather than resuming the undispatched remainder (the would-be chunk 3, another
    // 50cm) of the failed sequence -- proven by scripting a distinct new command (an unrelated
    // Elevate) and observing that it, not a 4th advance(), is what gets dispatched next.
    MappingStepCommand oversized_command;
    oversized_command.movement = advanceCommand(150.0 * isq::length[cm]);

    MappingStepCommand next_command;
    next_command.movement = elevateCommand(5.0 * isq::length[cm]);

    // Catch-alls declared first (lowest priority): the specific, ordered expectations below
    // (declared later = higher priority) claim their exact calls, so these only catch anything
    // else -- in particular a 5th advance() call, which would mean the leftover chunk 3 of the
    // failed sequence was wrongly resumed instead of a fresh command being fetched.
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    // Post-movement GPS (row 12) only needs to agree after a chunk that actually *succeeds*: the
    // failing chunk 2 never reaches row-12 validation at all. Chunk 1's advance(50cm) lands the
    // internal estimate at x=50; the pre-step readings surrounding the failed chunk 2 and the
    // fresh elevate command are otherwise unused, so they stay at the origin; the final
    // elevate(5cm) is expected to land at (50, 0, 5).
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{},
             Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 5.0 * z_extent[cm]}})));

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(oversized_command));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm]))
        .Times(3)
        .WillRepeatedly(Return(MovementResult{false, "blocked by obstacle"}));
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(next_command));
    EXPECT_CALL(movement_, elevate(5.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult chunk1 = control_.step();
    EXPECT_EQ(chunk1.status, DroneStepStatus::Continue);
    ASSERT_EQ(control_.state().step_index, 1u);

    EXPECT_THROW((void)control_.step(), std::runtime_error)
        << "chunk 2: 3 exhausted retries against the same 50cm chunk throw";
    EXPECT_EQ(control_.state().step_index, 1u)
        << "step_index_ must not be incremented for the failing chunk";

    const DroneStepResult after_failure = control_.step();
    EXPECT_EQ(after_failure.status, DroneStepStatus::Continue)
        << "must fetch and dispatch a fresh command from the algorithm, not resume the "
           "undispatched remainder of the failed oversized command";
    EXPECT_EQ(control_.state().step_index, 2u);
}

TEST_F(DroneControl, AdvanceWithinMaxRemainsUnchangedInOneChunk) {
    // 30cm is within droneConfig()'s 50cm max_advance: dispatched as-is, in a single step(), with
    // no splitting. Post-movement GPS (row 12) must agree with x=30cm.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{30.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(30.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(30.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 1u);
}

TEST_F(DroneControl, AdvanceSplitByDecimalMaxProducesExactChunkCountNoPhantomTrailingChunk) {
    // Regression for a floating-point residue bug in splitMagnitude(): repeated subtraction of a
    // decimal max (0.1cm) from a decimal total (1.0cm) leaves a tiny nonzero residue (order
    // 1e-16cm) after the 10th subtraction, since 0.1 isn't exactly representable in binary
    // floating point -- semantically this must still produce exactly 10 chunks, not an 11th,
    // near-zero-movement phantom chunk/step. Checked two ways: (a) every movement_.advance() call
    // is exactly 0.1cm, exactly 10 times (the catch-all below would catch any other value, e.g. a
    // phantom ~1e-16cm 11th call); (b) the original command's Finished status appears on exactly
    // the 10th step(), which is only possible if the movement queue is empty by then -- if an 11th
    // chunk had materialized, this step would still report Continue.
    DroneConfigData decimal_max_config = droneConfig();
    decimal_max_config.max_advance = 0.1 * isq::length[cm];
    DroneControlImpl decimal_control{
        decimal_max_config, missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    // Post-movement GPS (row 12) must agree with the cumulative x after each of the 10 chunks;
    // row 12's numerical tolerance absorbs the sub-ULP residue splitMagnitude()'s own repeated
    // subtraction leaves on the final chunk (see the comment above), so plain 0.1cm increments
    // are exact enough here.
    std::vector<Position3D> gps_positions;
    double cumulative_cm = 0.0;
    for (int i = 0; i < 10; ++i) {
        gps_positions.push_back(
            Position3D{cumulative_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
        cumulative_cm += 0.1;
        gps_positions.push_back(
            Position3D{cumulative_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    }
    ON_CALL(gps_, position()).WillByDefault(Invoke(gpsPositionSequence(std::move(gps_positions))));

    MappingStepCommand command;
    command.movement = advanceCommand(1.0 * isq::length[cm]);
    command.status = AlgorithmStatus::Finished;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, advance(0.1 * isq::length[cm]))
        .Times(10)
        .WillRepeatedly(Return(MovementResult{true, ""}));

    for (int i = 0; i < 9; ++i) {
        EXPECT_EQ(decimal_control.step().status, DroneStepStatus::Continue);
    }
    const DroneStepResult final_result = decimal_control.step();
    EXPECT_EQ(final_result.status, DroneStepStatus::Completed);
    EXPECT_EQ(final_result.message, "mapping finished");
    EXPECT_EQ(decimal_control.state().step_index, 10u);
}

TEST_F(DroneControl, LargeMaxWithSmallGenuineRemainderIsNotDroppedAsResidue) {
    // Regression for splitMagnitude()'s residual-tolerance formula: a fixed tolerance relative to
    // max_magnitude (e.g. 1e-9 * max_magnitude) scales the wrong way -- with max_advance=1e9cm it
    // would become 1cm, large enough to silently swallow a real 0.5cm remainder as if it were
    // floating-point noise. 1e9 + 0.5 split by a max of 1e9 must produce chunks 1e9 and 0.5, not
    // just 1e9cm alone.
    DroneConfigData large_max_config = droneConfig();
    large_max_config.max_advance = 1.0e9 * isq::length[cm];
    DroneControlImpl large_max_control{
        large_max_config, missionConfig(), lidar_, gps_, movement_, output_map_, algorithm_};

    // The movement below (~1e9cm) is far beyond the fixture's default unboundedMapConfig()
    // (+-100000cm); widen the map bounds here so it stays legal and the OOB amendment leaves it
    // unchanged, isolating this test to oversized-command splitting alone.
    stubMapBounds(output_map_, MappingBounds{-1.0e12 * x_extent[cm], 1.0e12 * x_extent[cm],
                                              -1.0e12 * y_extent[cm], 1.0e12 * y_extent[cm],
                                              -1.0e12 * z_extent[cm], 1.0e12 * z_extent[cm]});

    // Post-movement GPS (row 12) must agree with the cumulative x after each chunk (1e9, then
    // 1e9 + 0.5); row 12's tolerance scales with magnitude, so it comfortably covers rounding at
    // this scale without needing voxel/gps_resolution-sized slack.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{1.0e9 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{},
             Position3D{(1.0e9 + 0.5) * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(1.0e9 * isq::length[cm] + 0.5 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(1.0e9 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(0.5 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(large_max_control.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(large_max_control.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(large_max_control.state().step_index, 2u);
}

// --- Optional Common-Issues row 11: out-of-bound GPS readings ------------------------------
//
// "The GPS returns out-of-bound coordinates" -> compared against a self-maintained internal
// position estimate (initialized from the first-ever in-bounds GPS reading): if the internal
// estimate is also out of bounds, throw (nothing plausible to fall back on); otherwise the
// reading is a spurious sample and is ignored outright -- no Algorithm call, no movement, no
// scan/map write -- though it still counts as a step. output_map_.isInBounds() is the sole
// authority, exactly as it is for row-10's amendment logic.

TEST_F(DroneControl, FirstInBoundsGpsReadingInitializesInternalBaseline) {
    // The very first GPS reading is in bounds and becomes the internal position baseline.
    // Verified indirectly: a *later* out-of-bounds reading is judged against that baseline and
    // ignored rather than thrown -- which could only happen if a baseline had actually been
    // established from this very first reading (see
    // LaterOutOfBoundsGpsWithInBoundsInternalBaselineIsIgnored below for that behavior in
    // isolation, using its own freshly-initialized baseline).
    stubMapBounds(output_map_, MappingBounds{-50.0 * x_extent[cm], 50.0 * x_extent[cm],
                                              -50.0 * y_extent[cm], 50.0 * y_extent[cm],
                                              -50.0 * z_extent[cm], 50.0 * z_extent[cm]});
    const Position3D in_bounds_position{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    const Position3D out_of_bounds_position{1000.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(in_bounds_position))
        .WillRepeatedly(Return(out_of_bounds_position));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    // A scan (rather than a bare no-movement/no-scan command) keeps this a legitimate step
    // instead of a faulty NOOP (Optional Common-Issues row 4). Unlike a movement (even Hover),
    // attaching a scan doesn't add a row-12 post-movement gps_.position() re-read, so it doesn't
    // disturb this test's single-read-per-step GPS sequencing below.
    MappingStepCommand idle_command;
    idle_command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(idle_command));

    const DroneStepResult first = control_.step();
    EXPECT_EQ(first.status, DroneStepStatus::Continue);

    DroneStepResult second;
    EXPECT_NO_THROW(second = control_.step());
    EXPECT_EQ(second.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 2u);
}

TEST_F(DroneControl, FirstEverGpsReadingOutOfBoundsThrows) {
    // No internal baseline exists yet on the very first step() call: an out-of-bounds GPS reading
    // has nothing to be validated against, so it must throw rather than silently continuing or
    // substituting some other value.
    stubMapBounds(output_map_, MappingBounds{-50.0 * x_extent[cm], 50.0 * x_extent[cm],
                                              -50.0 * y_extent[cm], 50.0 * y_extent[cm],
                                              -50.0 * z_extent[cm], 50.0 * z_extent[cm]});
    EXPECT_CALL(gps_, position())
        .WillRepeatedly(Return(Position3D{1000.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));

    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(0);
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    EXPECT_THROW((void)control_.step(), std::runtime_error);
}

TEST_F(DroneControl, LaterOutOfBoundsGpsWithInBoundsInternalBaselineIsIgnored) {
    stubMapBounds(output_map_, MappingBounds{-50.0 * x_extent[cm], 50.0 * x_extent[cm],
                                              -50.0 * y_extent[cm], 50.0 * y_extent[cm],
                                              -50.0 * z_extent[cm], 50.0 * z_extent[cm]});
    const Position3D in_bounds_position{};
    const Position3D out_of_bounds_position{1000.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(in_bounds_position))
        .WillRepeatedly(Return(out_of_bounds_position));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    // A scan (rather than a bare no-movement/no-scan command) keeps this a legitimate step
    // instead of a faulty NOOP (Optional Common-Issues row 4). Unlike a movement (even Hover),
    // attaching a scan doesn't add a row-12 post-movement gps_.position() re-read, so it doesn't
    // disturb this test's single-read-per-step GPS sequencing below.
    MappingStepCommand idle_command;
    idle_command.scan_orientation = Orientation{};
    // Exactly once: the Algorithm must not be consulted again for the ignored step below.
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(idle_command));

    (void)control_.step(); // initializes the internal baseline in bounds

    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 2u);
}

TEST_F(DroneControl, RepeatedIgnoredOutOfBoundsGpsReadingsConsumeSteps) {
    // Each ignored sample still counts as a step, so repeated bad readings consume the
    // MissionControl max_steps budget instead of looping forever.
    stubMapBounds(output_map_, MappingBounds{-50.0 * x_extent[cm], 50.0 * x_extent[cm],
                                              -50.0 * y_extent[cm], 50.0 * y_extent[cm],
                                              -50.0 * z_extent[cm], 50.0 * z_extent[cm]});
    const Position3D in_bounds_position{};
    const Position3D out_of_bounds_position{1000.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(in_bounds_position))
        .WillRepeatedly(Return(out_of_bounds_position));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    // A scan (rather than a bare no-movement/no-scan command) keeps this a legitimate step
    // instead of a faulty NOOP (Optional Common-Issues row 4). Unlike a movement (even Hover),
    // attaching a scan doesn't add a row-12 post-movement gps_.position() re-read, so it doesn't
    // disturb this test's single-read-per-step GPS sequencing below.
    MappingStepCommand idle_command;
    idle_command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(idle_command));

    (void)control_.step();
    ASSERT_EQ(control_.state().step_index, 1u);

    for (int i = 0; i < 5; ++i) {
        const DroneStepResult result = control_.step();
        EXPECT_EQ(result.status, DroneStepStatus::Continue);
    }
    EXPECT_EQ(control_.state().step_index, 6u);
}

TEST_F(DroneControl, OutOfBoundsGpsPreservesPendingRow8SequenceAndResumesSameChunk) {
    // A row-8 oversized-Advance sequence is mid-flight (chunk 1 of 3 dispatched); the next
    // step()'s pre-step GPS sample is spuriously out of bounds while the internal baseline
    // remains in bounds -- it must be ignored without discarding the two chunks still pending, so
    // a later valid reading resumes the very same sequence (chunk 2) rather than the Algorithm
    // being consulted again.
    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]); // droneConfig() max_advance=50cm -> 3 chunks
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm]))
        .Times(2)
        .WillRepeatedly(Return(MovementResult{true, ""}));

    const Position3D origin{};
    const Position3D after_chunk1{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    const Position3D out_of_bounds{1.0e9 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    const Position3D after_chunk2{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(origin))              // step1 pre-step: initializes the baseline
        .WillOnce(Return(after_chunk1))        // step1 post-movement (row 12): matches
        .WillOnce(Return(out_of_bounds))       // step2 pre-step: spurious OOB, ignored (row 11)
        .WillOnce(Return(after_chunk1))        // step3 pre-step: back in bounds
        .WillOnce(Return(after_chunk2))        // step3 post-movement (row 12): matches
        .WillRepeatedly(Return(after_chunk2));

    const DroneStepResult chunk1 = control_.step();
    EXPECT_EQ(chunk1.status, DroneStepStatus::Continue);

    const DroneStepResult ignored = control_.step();
    EXPECT_EQ(ignored.status, DroneStepStatus::Continue);

    const DroneStepResult chunk2 = control_.step();
    EXPECT_EQ(chunk2.status, DroneStepStatus::Continue)
        << "must resume chunk 2 of the pending sequence, not fetch a new command from the "
           "Algorithm (which is scripted to be called only once)";
    EXPECT_EQ(control_.state().step_index, 3u)
        << "the ignored GPS sample still counted as a step";
}

TEST_F(DroneControl, OutOfBoundsGpsWithOutOfBoundsInternalBaselineThrows) {
    // Even the internal baseline itself is currently out of bounds: there is no plausible
    // reference left to call the new reading spurious, so it must throw.
    stubMapBounds(output_map_, MappingBounds{-50.0 * x_extent[cm], 50.0 * x_extent[cm],
                                              -50.0 * y_extent[cm], 50.0 * y_extent[cm],
                                              -50.0 * z_extent[cm], 50.0 * z_extent[cm]});
    const Position3D in_bounds_position{};
    const Position3D out_of_bounds_position{1000.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(in_bounds_position))
        .WillRepeatedly(Return(out_of_bounds_position));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    // A scan (rather than a bare no-movement/no-scan command) keeps this a legitimate step
    // instead of a faulty NOOP (Optional Common-Issues row 4). Unlike a movement (even Hover),
    // attaching a scan doesn't add a row-12 post-movement gps_.position() re-read, so it doesn't
    // disturb this test's single-read-per-step GPS sequencing below.
    MappingStepCommand idle_command;
    idle_command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(idle_command));

    (void)control_.step(); // baseline initialized at the origin, in bounds under the current map

    // The map shrinks so that the *existing* internal baseline is now itself out of bounds too.
    stubMapBounds(output_map_, MappingBounds{100.0 * x_extent[cm], 200.0 * x_extent[cm],
                                              100.0 * y_extent[cm], 200.0 * y_extent[cm],
                                              100.0 * z_extent[cm], 200.0 * z_extent[cm]});

    EXPECT_THROW((void)control_.step(), std::runtime_error);
}

// --- Optional Common-Issues row 12: post-movement GPS inconsistency ------------------------
//
// "A movement executed but GPS updated with impossible coordinates" -> after every successfully
// executed movement chunk (even one with no scan requested), the internal position estimate
// advances to the physically-expected position and a fresh GPS reading is compared against it
// (a purely numerical tolerance, not a voxel/gps_resolution-sized one). A mismatch retries GPS up
// to 3 further times; if none match, the step reports Error, performs no scan, discards any
// remaining row-8 chunks, and leaves the internal estimate at the expected (not GPS-reported)
// position.

TEST_F(DroneControl, SuccessfulMovementTriggersPostMovementGpsValidationEvenWithoutScan) {
    // No scan_orientation is set on this command, yet a movement still executes -- row 12's
    // post-movement GPS check must still run and can still fail the step, proving it is not
    // gated on a scan having been requested.
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]); // expected x=10, no scan
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    // GPS never reports the expected (10,0,0): every reading is the stale pre-movement origin.
    EXPECT_CALL(gps_, position()).WillRepeatedly(Return(Position3D{}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
}

TEST_F(DroneControl, PostMovementGpsMatchingExpectedSucceedsWithoutScan) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    const Position3D expected{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))
        .WillRepeatedly(Return(expected));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, InBoundsButWrongPostMovementGpsTriggersRow12Retries) {
    // A plausible, in-bounds-but-wrong reading (5cm instead of the expected 10cm) still drives
    // row 12's retry loop -- proving the mismatch alone, not out-of-boundsness, is what triggers
    // it. Times(5) pins the exact call count: 1 pre-step read + 4 post-movement attempts (the
    // initial read plus all 3 retries).
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]); // expected x=10
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const Position3D wrong_in_bounds{5.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .Times(5)
        .WillOnce(Return(Position3D{}))
        .WillRepeatedly(Return(wrong_in_bounds));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
}

TEST_F(DroneControl, OutOfBoundsWrongPostMovementGpsFollowsRow12NotRow11) {
    // The expected post-movement position (10,0,0) is itself in bounds, but every GPS reading
    // after the movement is both wrong *and* out of bounds. This must still follow row 12's
    // retry-then-Error path (and must not throw): row 11's throw is reserved for the *pre-step*
    // check, never triggered by a post-movement mismatch.
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 1000.0 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const Position3D out_of_bounds{1.0e9 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))       // pre-step: in bounds
        .WillRepeatedly(Return(out_of_bounds)); // every post-movement attempt: wrong and OOB

    DroneStepResult result;
    EXPECT_NO_THROW(result = control_.step());
    EXPECT_EQ(result.status, DroneStepStatus::Error);
}

TEST_F(DroneControl, BadFirstPostMovementReadingFollowedByValidRetrySucceeds) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const Position3D expected{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))  // pre-step
        .WillOnce(Return(Position3D{}))  // post-movement attempt 1: wrong
        .WillOnce(Return(expected))      // retry 1: matches
        .WillRepeatedly(Return(expected));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AllPostMovementGpsRetriesFailingReturnsErrorWithoutScanningOrWriting) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{}; // a scan was requested
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(gps_, position()).WillRepeatedly(Return(Position3D{})); // never reports expected x=10

    EXPECT_CALL(lidar_, scan(_)).Times(0);
    EXPECT_CALL(output_map_, set(_, _)).Times(0);

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(control_.state().step_index, 0u)
        << "the existing Error-path convention (no scan/write, step_index_ untouched) must hold";
}

TEST_F(DroneControl, SuccessfulRetryFollowedByScanUsesTheAcceptedGpsPosition) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const Position3D wrong_first_reading{};
    const Position3D accepted{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{}))        // pre-step
        .WillOnce(Return(wrong_first_reading)) // post-movement attempt 1: wrong (!= expected x=10)
        .WillOnce(Return(accepted))            // retry 1: matches, accepted
        .WillRepeatedly(Return(accepted));

    LidarHit hit;
    hit.distance = 5.0 * isq::length[cm];
    hit.angle = Orientation{};
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{hit}));
    EXPECT_CALL(lidar_, config()).WillRepeatedly(Return(lidarConfig()));
    EXPECT_CALL(output_map_, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Unmapped));

    const MapConfig map_config{
        MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm], -1000.0 * y_extent[cm],
                      1000.0 * y_extent[cm], -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]},
        Position3D{}, 10.0 * isq::length[cm]};
    EXPECT_CALL(output_map_, getMapConfig()).WillRepeatedly(Return(map_config));
    EXPECT_CALL(output_map_, isInBounds(_)).WillRepeatedly(Return(true));

    // A hit landing near the *accepted* retried position (10cm + 5cm hit = 15cm), not near the
    // wrong first reading (0cm + 5cm = 5cm), proves the scan used the retried, accepted GPS
    // position rather than the initial mismatched one.
    auto isNearX = [](double expected_cm) {
        return [expected_cm](const Position3D& p) {
            return std::abs(p.x.force_numerical_value_in(cm) - expected_cm) < 1.0;
        };
    };
    EXPECT_CALL(output_map_, set(_, _)).Times(testing::AnyNumber());
    EXPECT_CALL(output_map_, set(Truly(isNearX(15.0)), VoxelOccupancy::Occupied)).Times(1);
    EXPECT_CALL(output_map_, set(Truly(isNearX(5.0)), VoxelOccupancy::Occupied)).Times(0);

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, PostMovementGpsWithOnlyFloatingPointRoundingDifferenceIsAccepted) {
    // A post-movement GPS reading that differs from the expected position only by an
    // ordinary-double-rounding-scale residue must be accepted without retrying: row 12's
    // tolerance is explicitly numerical, not voxel/gps_resolution-sized (contrast with
    // InBoundsButWrongPostMovementGpsTriggersRow12Retries above, where a real 5cm discrepancy is
    // rejected).
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]); // expected x=10
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const auto tiny_residue = 1.0e-10 * x_extent[cm]; // far below any voxel/gps_resolution scale
    EXPECT_CALL(gps_, position())
        .Times(2)
        .WillOnce(Return(Position3D{}))
        .WillOnce(Return(Position3D{10.0 * x_extent[cm] + tiny_residue, 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue)
        << "an exact-Times(2) call count also proves no retry was triggered";
}

TEST_F(DroneControl, IntermediateRow8ChunkGpsFailureDiscardsRemainingSequenceAfterAllRetriesFail) {
    // Chunk 1 of a 3-chunk oversized Advance succeeds and passes row-12 validation; chunk 2 also
    // physically succeeds but its post-movement GPS never matches the expected position through
    // all retries -- proving the remaining chunk 3 is discarded (a further step() call fetches a
    // brand-new command from the Algorithm rather than resuming chunk 3) only *after* chunk 2's
    // movement executed and its GPS retries were exhausted, not merely because the sequence was
    // mid-flight.
    MappingStepCommand oversized_command;
    oversized_command.movement = advanceCommand(150.0 * isq::length[cm]); // 50 + 50 + 50

    MappingStepCommand next_command;
    next_command.movement = elevateCommand(5.0 * isq::length[cm]);

    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    // Catch-all (lowest priority): a 3rd advance() call would mean chunk 3 was wrongly resumed.
    EXPECT_CALL(movement_, advance(_)).Times(0);

    const Position3D after_chunk1{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    // Chunk 2's expected position is (100,0,0); every post-movement reading instead stays stuck
    // at chunk 1's position, exhausting all retries. internal_position_ is then left at chunk 2's
    // expected (100,0,0), so the fresh elevate command's own expected position is (100,0,5).
    // Declared as ON_CALL (rather than EXPECT_CALL), and before the InSequence below, so it never
    // gets swept into that algorithm_/movement_-only call ordering.
    const Position3D final_expected{100.0 * x_extent[cm], 0.0 * y_extent[cm], 5.0 * z_extent[cm]};
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence({
            Position3D{},       // step1 pre-step
            after_chunk1,       // step1 post-movement: matches
            after_chunk1,       // step2 pre-step (in bounds; value otherwise unused)
            after_chunk1,       // step2 post-movement attempt 1: wrong
            after_chunk1,       // retry 1: wrong
            after_chunk1,       // retry 2: wrong
            after_chunk1,       // retry 3: wrong -- exhausted, Error
            after_chunk1,       // step3 pre-step (fresh command; in bounds)
            final_expected,     // step3 post-movement: matches
        })));

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(oversized_command));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(next_command));
    EXPECT_CALL(movement_, elevate(5.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult chunk1 = control_.step();
    EXPECT_EQ(chunk1.status, DroneStepStatus::Continue);

    const DroneStepResult chunk2 = control_.step();
    EXPECT_EQ(chunk2.status, DroneStepStatus::Error);

    const DroneStepResult after_failure = control_.step();
    EXPECT_EQ(after_failure.status, DroneStepStatus::Continue)
        << "must fetch and dispatch a fresh command from the Algorithm, not resume the "
           "undispatched chunk 3 of the GPS-inconsistent sequence";
}

// --- Optional Common-Issues row 12 follow-up: tolerance and in-bounds fixes ----------------
//
// Two focused corrections to row 12's post-movement GPS validation:
//  (1) the numerical tolerance must scale with machine epsilon, not a fixed relative fraction of
//      magnitude -- the latter tolerates a physically real discrepancy (e.g. ~1cm at a 1e9cm
//      coordinate) as if it were mere rounding noise;
//  (2) an accepted reading must also be in bounds -- row 10 already guarantees the dispatched
//      destination is legal, so an out-of-bounds reading can never be the true post-movement
//      position no matter how numerically close it is to what was expected.

TEST_F(DroneControl, LargeMagnitudeGpsToleranceRejectsARealMismatch) {
    // At a coordinate around 1e9cm, a fixed relative tolerance (e.g. 1e-9 * magnitude) would
    // already tolerate close to 1cm of real error. The machine-epsilon-based tolerance must
    // reject a genuine 0.5cm discrepancy at this scale regardless of retries.
    stubMapBounds(output_map_, MappingBounds{-1.0e12 * x_extent[cm], 1.0e12 * x_extent[cm],
                                              -1.0e12 * y_extent[cm], 1.0e12 * y_extent[cm],
                                              -1.0e12 * z_extent[cm], 1.0e12 * z_extent[cm]});

    const double start_cm = 1.0e9;
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]); // expected x = start_cm + 10
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const double expected_cm = start_cm + 10.0;
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{start_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}))
        .WillRepeatedly(Return(Position3D{
            (expected_cm + 0.5) * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
}

TEST_F(DroneControl, LargeMagnitudeGpsToleranceAcceptsGenuineFloatingPointResidue) {
    // Same scale as above, but the discrepancy is sized to what double rounding could actually
    // produce there (a handful of ULPs) rather than a real physical error -- this must be
    // accepted without retrying.
    stubMapBounds(output_map_, MappingBounds{-1.0e12 * x_extent[cm], 1.0e12 * x_extent[cm],
                                              -1.0e12 * y_extent[cm], 1.0e12 * y_extent[cm],
                                              -1.0e12 * z_extent[cm], 1.0e12 * z_extent[cm]});

    const double start_cm = 1.0e9;
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const double expected_cm = start_cm + 10.0;
    const double residue_cm = 4.0 * std::numeric_limits<double>::epsilon() * expected_cm;
    EXPECT_CALL(gps_, position())
        .Times(2)
        .WillOnce(Return(Position3D{start_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}))
        .WillOnce(Return(Position3D{
            (expected_cm + residue_cm) * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue)
        << "an exact-Times(2) call count also proves no retry was triggered";
}

TEST_F(DroneControl, OutOfBoundsPostMovementGpsCloseToExpectedIsNotAcceptedOrUsedToResyncInternalPosition) {
    // A post-movement GPS reading can be numerically indistinguishable from the expected position
    // (within row 12's tolerance) and still be physically impossible: row 10 already guarantees
    // the dispatched destination is legal, so a reading that fails output_map_.isInBounds() must
    // never be accepted merely because it is numerically close. The y bound below is drawn
    // deliberately tight (just above the drone's actual y=0) so a y reading that is itself out of
    // bounds still falls well inside row 12's numerical tolerance of the expected y=0 -- and the
    // movement's own x-destination stays comfortably inside the generous x/z bounds, so row 10
    // never amends it, isolating this test to row 12's own in-bounds check.
    stubMapBounds(output_map_, MappingBounds{-1000.0 * x_extent[cm], 1000.0 * x_extent[cm],
                                              -1000.0 * y_extent[cm], 5.0e-10 * y_extent[cm],
                                              -1000.0 * z_extent[cm], 1000.0 * z_extent[cm]});

    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]); // expected (10, 0, 0)
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);
    EXPECT_CALL(output_map_, set(_, _)).Times(0);

    // Every post-movement attempt reports a y of 8e-10cm: outside the y bound above (>= 5e-10cm),
    // yet within row 12's numerical tolerance of the expected y=0.
    const Position3D close_but_out_of_bounds{
        10.0 * x_extent[cm], 8.0e-10 * y_extent[cm], 0.0 * z_extent[cm]};
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(Position3D{})) // pre-step
        .WillRepeatedly(Return(close_but_out_of_bounds));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error)
        << "a numerically-close-but-out-of-bounds reading must still be rejected -- and since "
           "step() only ever assigns internal_position_ from a *validated* reading, this Error "
           "also proves it was never resynced to this out-of-bounds value";
}

// --- Optional Common-Issues rows 3 & 4: invalid-value / NOOP algorithm retries -------------
//
// Row 3 ("A faulty algorithm returned a command with invalid values"): only non-finite ACTIVE
// numeric fields are rejected -- Rotate's angle; Advance/Elevate's distance;
// scan_orientation's horizontal/altitude -- never negative/zero/oversized-but-finite values,
// unused MovementCommand fields, or enum values. Row 4 ("The algorithm returned a NOOP"): only
// a Working status with neither movement nor scan is faulty; Finished/FinishedWithUnmappableVoxels
// with no movement/scan remain legitimate terminal results. Both rows share one budget of 3 total
// mapping_algorithm_.nextStep() calls per requested command -- the same DroneState/step_index and
// latest_scan for every attempt -- and exhausting it throws. This all happens only when a *new*
// command is being requested (prepareNextSequence()); a pending row-8 sequence never re-enters it
// (see PendingRow8SequenceDoesNotReconsultAlgorithmOrApplyRetryValidation below).

TEST_F(DroneControl, RotateWithNonFiniteAngleIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand invalid_command;
    invalid_command.movement =
        rotateCommand(std::numeric_limits<double>::quiet_NaN() * horizontal_angle[deg]);

    MappingStepCommand valid_command;
    valid_command.movement = rotateCommand(10.0 * horizontal_angle[deg]);

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 10.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 1u)
        << "the rejected attempt must not consume a separate step -- only the eventual valid "
           "command's own dispatch does";
}

TEST_F(DroneControl, AdvanceWithInfiniteDistanceIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand invalid_command;
    invalid_command.movement = advanceCommand(std::numeric_limits<double>::infinity() * isq::length[cm]);

    MappingStepCommand valid_command;
    valid_command.movement = advanceCommand(20.0 * isq::length[cm]);

    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{20.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(movement_, advance(20.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ElevateWithNaNDistanceIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand invalid_command;
    invalid_command.movement = elevateCommand(std::numeric_limits<double>::quiet_NaN() * isq::length[cm]);

    MappingStepCommand valid_command;
    valid_command.movement = elevateCommand(12.0 * isq::length[cm]);

    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 12.0 * z_extent[cm]}})));

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(movement_, elevate(12.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ScanOrientationWithNonFiniteHorizontalIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand invalid_command;
    invalid_command.scan_orientation =
        Orientation{std::numeric_limits<double>::quiet_NaN() * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    const Orientation valid_orientation{30.0 * horizontal_angle[deg], 5.0 * altitude_angle[deg]};
    MappingStepCommand valid_command;
    valid_command.scan_orientation = valid_orientation;

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(lidar_, scan(OrientationEq(valid_orientation))).WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ScanOrientationWithInfiniteAltitudeIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand invalid_command;
    invalid_command.scan_orientation = Orientation{
        0.0 * horizontal_angle[deg], std::numeric_limits<double>::infinity() * altitude_angle[deg]};

    const Orientation valid_orientation{15.0 * horizontal_angle[deg], -10.0 * altitude_angle[deg]};
    MappingStepCommand valid_command;
    valid_command.scan_orientation = valid_orientation;

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(lidar_, scan(OrientationEq(valid_orientation))).WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, NegativeAdvanceDistanceDoesNotTriggerRetry) {
    // Negative-but-finite values are never row-3 invalid (only Row 10's OOB handling and Row 8's
    // splitting may amend them) -- the algorithm must be consulted exactly once.
    MappingStepCommand command;
    command.movement = advanceCommand(-15.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{-15.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    EXPECT_CALL(movement_, advance(-15.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ZeroRotateAngleDoesNotTriggerRetry) {
    // Zero is finite and therefore never row-3 invalid.
    MappingStepCommand command;
    command.movement = rotateCommand(0.0 * horizontal_angle[deg]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 0.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, ThreeInvalidCommandsInARowThrows) {
    MappingStepCommand invalid_command;
    invalid_command.movement = advanceCommand(std::numeric_limits<double>::quiet_NaN() * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(3).WillRepeatedly(Return(invalid_command));

    EXPECT_CALL(movement_, advance(_)).Times(0);

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u)
        << "no attempt -- rejected or otherwise -- may consume a step before the throw";
}

TEST_F(DroneControl, WorkingEmptyCommandIsRejectedAndValidRetrySucceeds) {
    MappingStepCommand noop_command; // Working, no movement, no scan -- the faulty row-4 NOOP
    MappingStepCommand valid_command;
    valid_command.movement = hoverCommand();

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(noop_command))
        .WillOnce(Return(valid_command));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 1u)
        << "the rejected NOOP must not consume a separate step";
}

TEST_F(DroneControl, ThreeWorkingNoopCommandsInARowThrows) {
    MappingStepCommand noop_command; // Working, no movement, no scan, three times running
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(3).WillRepeatedly(Return(noop_command));

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u);
}

TEST_F(DroneControl, MixedInvalidThenNoopThenValidSucceedsOnThirdAttempt) {
    // Proves the two failure kinds (row 3 invalid values, row 4 NOOP) draw from the same shared
    // 3-attempt budget rather than each getting their own: 1 invalid + 1 NOOP + 1 valid, and the
    // 3rd attempt still succeeds.
    MappingStepCommand invalid_command;
    invalid_command.movement =
        rotateCommand(std::numeric_limits<double>::infinity() * horizontal_angle[deg]);
    MappingStepCommand noop_command; // Working, no movement, no scan
    MappingStepCommand valid_command;
    valid_command.movement = rotateCommand(15.0 * horizontal_angle[deg]);

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(noop_command))
        .WillOnce(Return(valid_command));
    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 15.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, MixedInvalidAndNoopExhaustingSharedBudgetThrows) {
    // Same shared-budget proof as above, but all 3 attempts are rejected (a mix of both kinds),
    // exhausting the budget without ever succeeding.
    MappingStepCommand invalid_command;
    invalid_command.movement = advanceCommand(std::numeric_limits<double>::quiet_NaN() * isq::length[cm]);
    MappingStepCommand noop_command; // Working, no movement, no scan

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(invalid_command))
        .WillOnce(Return(noop_command))
        .WillOnce(Return(invalid_command));

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u);
}

TEST_F(DroneControl, NoopThenFinishedWithNoMovementOrScanSucceedsAsValidTerminalResult) {
    // Status is what distinguishes a faulty NOOP from a legitimate terminal result (Row 4):
    // a Finished command with no movement/scan is accepted immediately, even right after a
    // rejected Working NOOP -- exercising the retry and the terminal-status exemption together.
    MappingStepCommand noop_command; // Working, no movement, no scan -- rejected
    MappingStepCommand finished_command;
    finished_command.status = AlgorithmStatus::Finished; // no movement, no scan -- legitimate

    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(noop_command))
        .WillOnce(Return(finished_command));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Completed);
    EXPECT_EQ(result.message, "mapping finished");
}

TEST_F(DroneControl, FinishedWithUnmappableVoxelsWithNoMovementOrScanIsNotTreatedAsNoop) {
    // Same terminal-status exemption for the other non-Working status, checked directly (no
    // retry involved): the algorithm must be consulted exactly once.
    MappingStepCommand command; // no movement, no scan
    command.status = AlgorithmStatus::FinishedWithUnmappableVoxels;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Completed);
    EXPECT_EQ(result.message, "mapping finished with unmappable voxels remaining");
}

TEST_F(DroneControl, PendingRow8SequenceDoesNotReconsultAlgorithmOrApplyRetryValidation) {
    // While a row-8 oversized-movement sequence is mid-flight, prepareNextSequence() -- and thus
    // the row-3/4 retry loop -- must not run again: only the very first of these 3 chunks comes
    // from an algorithm call at all, proven by the Times(1) below spanning all 3 step() calls.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
             Position3D{}, Position3D{150.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]); // droneConfig() max_advance=50cm -> 3 chunks
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue);
}

// --- Optional Common-Issues row 6: empty LiDAR scan retries ---------------------------------
//
// "The LiDAR returns an empty vector" -> retry only the scan itself (same requested orientation),
// up to kMaxLidarScanAttempts (3) total lidar_.scan() calls, before giving up. Never re-dispatches
// the movement that may have preceded the scan, and never re-consults the Algorithm -- both of
// those happen at most once per step(), same as before this row existed. Only applies when a scan
// was actually requested; "no scan requested" is never treated as a LiDAR failure (proven
// implicitly by every other test in this file that requests no scan and never throws).

TEST_F(DroneControl, EmptyThenNonEmptyLidarScanSucceedsOnRetry) {
    const Orientation orientation{20.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MappingStepCommand command;
    command.scan_orientation = orientation;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(OrientationEq(orientation)))
        .WillOnce(Return(LidarScanResult{}))
        .WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 1u)
        << "the rejected empty attempt must not consume a separate step";
}

TEST_F(DroneControl, ThreeEmptyLidarScansInARowThrows) {
    // Also proves the Phase-2B scan/map-processing exception catches (see the
    // LidarScanThrowing.../ApplyToMapThrowing... tests below) do not swallow this throw: no
    // exception is involved here (every attempt returns normally, just empty), and it must still
    // propagate all the way out of step() as std::runtime_error.
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(_)).Times(3).WillRepeatedly(Return(LidarScanResult{}));

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u)
        << "no attempt -- rejected or otherwise -- may consume a step before the throw";
}

// --- Phase 2B fix 1: scan/map-processing exceptions in DroneControlImpl::step() ------------
//
// lidar_.scan() and ScanResultToVoxels::applyToMap() (map access/atVoxel/set) can throw for
// reasons unrelated to Row 6's "empty vector" case -- e.g. the underlying map itself faulting.
// Each is now caught locally and reported as DroneStepStatus::Error, exactly like the existing
// movement-exception path: step_index_ is never incremented and latest_scan_ is never touched
// (never overwritten by a scan that only partially succeeded or never applied). Row 6's own
// exhaustion throw (proven above) is untouched -- these new catches are scoped to only the
// lidar_.scan() call and only the applyToMap() call, never wrapping the empty-result retry loop
// itself.

TEST_F(DroneControl, LidarScanThrowingReturnsErrorWithoutUpdatingStepIndexOrLatestScan) {
    LidarHit hit;
    hit.distance = 10.0 * isq::length[cm];
    const LidarScanResult previous_scan{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isSingleHitScan = [](const LidarScanResult* scan) { return scan != nullptr && scan->size() == 1; };

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(previous_scan));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce([]() -> LidarScanResult {
        throw std::runtime_error("lidar hardware fault");
    });

    (void)control_.step(); // step 1: scans successfully, stores previous_scan
    ASSERT_EQ(control_.state().step_index, 1u);

    const DroneStepResult result = control_.step(); // step 2: lidar_.scan throws
    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(result.message, "lidar hardware fault");
    EXPECT_EQ(control_.state().step_index, 1u)
        << "step_index_ must not advance for a step whose scan threw";

    // step 3: the Algorithm must still see step 1's scan, proving latest_scan_ was never touched
    // by the throwing attempt.
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));
    (void)control_.step();
}

TEST_F(DroneControl, ApplyToMapThrowingReturnsErrorWithoutUpdatingStepIndexOrLatestScan) {
    LidarHit hit;
    hit.distance = 10.0 * isq::length[cm];
    const LidarScanResult previous_scan{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isSingleHitScan = [](const LidarScanResult* scan) { return scan != nullptr && scan->size() == 1; };

    // output_map_.isInBounds() is also called unconditionally at the top of every step() (row
    // 11), so it can't be scoped to only step 2's applyToMap call without disturbing row 11.
    // applyToMap's own next map access, output_map_.getMapConfig().resolution, is called exactly
    // once per applyToMap invocation and nowhere else for a scan-only (no movement) command --
    // throwing on exactly the 2nd call reproduces a map-access failure inside step 2's
    // applyToMap without disturbing step 1's (or step 3's) successful map write.
    auto call_count = std::make_shared<int>(0);
    ON_CALL(output_map_, getMapConfig()).WillByDefault(Invoke([call_count]() -> MapConfig {
        ++*call_count;
        if (*call_count == 2) {
            throw std::runtime_error("map access fault");
        }
        return unboundedMapConfig();
    }));

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(previous_scan));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));

    (void)control_.step(); // step 1: scans and maps successfully, stores previous_scan
    ASSERT_EQ(control_.state().step_index, 1u);

    const DroneStepResult result = control_.step(); // step 2: applyToMap throws
    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(result.message, "map access fault");
    EXPECT_EQ(control_.state().step_index, 1u)
        << "step_index_ must not advance for a step whose map write threw";

    // step 3: the Algorithm must still see step 1's scan, proving latest_scan_ was never
    // overwritten by the throwing attempt's scan, even though lidar_.scan() itself had already
    // succeeded before applyToMap threw.
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(nonEmptyScanResult()));
    (void)control_.step();
}

TEST_F(DroneControl, LidarScanExceptionDuringRetryLoopIsNotCountedAsRow6EmptyAttempt) {
    // A thrown exception on any retry attempt returns Error immediately -- it is never treated as
    // one of Row 6's "empty result" attempts, and the retry loop never continues past it: exactly
    // 2 lidar_.scan() calls happen here (empty, then throw), never a 3rd that Row 6's own
    // exhaustion would need.
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Return(LidarScanResult{})) // rejected: empty (row 6, attempt 1)
        .WillOnce([]() -> LidarScanResult { throw std::runtime_error("lidar hardware fault"); });

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(result.message, "lidar hardware fault");
}

TEST_F(DroneControl, MovementExecutesOnlyOnceWhileLidarScanRetries) {
    // Movement + scan requested together: the movement must be dispatched exactly once even
    // though the scan that follows it needs two attempts to succeed.
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .Times(1)
        .WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Return(LidarScanResult{}))
        .WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AlgorithmCalledOnlyOnceDuringLidarScanRetries) {
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Return(LidarScanResult{}))
        .WillOnce(Return(LidarScanResult{}))
        .WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue)
        << "the 3rd attempt succeeding, after 2 rejected empties, also proves the shared "
           "kMaxLidarScanAttempts budget allows a final-attempt success";
}

TEST_F(DroneControl, SuccessfulRetryScanIsMappedAndStoredAsLatestScan) {
    // Proves it's specifically the *accepted retry's* scan -- not the rejected empty one -- that
    // gets applied to output_map_ and stored as latest_scan_: the next step()'s Algorithm call
    // must see exactly this scan's content as latest_scan.
    LidarHit hit;
    hit.distance = 42.0 * isq::length[cm];
    const LidarScanResult retry_scan{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    MappingStepCommand idle_command;
    idle_command.movement = hoverCommand();

    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Return(LidarScanResult{})) // rejected: empty
        .WillOnce(Return(retry_scan));       // accepted: the retry

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isRetryScan = [](const LidarScanResult* scan) {
        return scan != nullptr && scan->size() == 1 &&
               scan->front().distance == 42.0 * isq::length[cm];
    };

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isRetryScan))).WillOnce(Return(idle_command));

    (void)control_.step(); // step 1: empty scan rejected, retry succeeds and is stored
    (void)control_.step(); // step 2: the Algorithm must see the retry's scan, not the empty one
}

// --- Optional Common-Issues row 7: movement driver returns false -----------------------------
//
// "The movement driver returns false" -> retry the exact same already-prepared chunk (post row-10
// amendment, post row-8 splitting) up to kMaxMovementAttempts (3) total driver calls, all within
// the same step() call, before giving up. Never re-consults the Algorithm, never advances to a
// later pending row-8 chunk, and never touches internal_position_/step_index_/scan state for a
// failed attempt -- only a genuinely successful attempt falls through to row 12 and the scan
// block. Exhausting the budget throws std::runtime_error (not an Error DroneStepResult). A thrown
// exception is a distinct, non-retried failure mode (see
// MovementExceptionReturnsErrorWithoutScanningOrAdvancingStepIndex above). Hover never calls
// movement_ at all, so it is unaffected by any of this.

TEST_F(DroneControl, FalseThenSuccessRetriesSameMovementOnceAndSucceeds) {
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .WillOnce(Return(MovementResult{false, "blocked"}))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
    EXPECT_EQ(control_.state().step_index, 1u)
        << "the rejected attempt must not consume a separate step -- only the eventual success "
           "does";
}

TEST_F(DroneControl, FalseFalseSuccessSucceedsOnThirdAttempt) {
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .WillOnce(Return(MovementResult{false, "blocked"}))
        .WillOnce(Return(MovementResult{false, "blocked again"}))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue)
        << "the 3rd attempt succeeding, after 2 rejected false results, proves the retry budget "
           "allows a final-attempt success";
}

TEST_F(DroneControl, FalseThreeTimesThrows) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .Times(3)
        .WillRepeatedly(Return(MovementResult{false, "blocked"}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    EXPECT_THROW((void)control_.step(), std::runtime_error);
    EXPECT_EQ(control_.state().step_index, 0u)
        << "no attempt -- rejected or otherwise -- may consume a step before the throw";
}

TEST_F(DroneControl, AlgorithmCalledOnlyOnceDuringMovementRetries) {
    MappingStepCommand command;
    command.movement = rotateCommand(10.0 * horizontal_angle[deg]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, rotate(RotationDirection::Left, 10.0 * horizontal_angle[deg]))
        .WillOnce(Return(MovementResult{false, "servo jammed"}))
        .WillOnce(Return(MovementResult{false, "servo jammed again"}))
        .WillOnce(Return(MovementResult{true, ""}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue)
        << "3 driver attempts against a single Algorithm-issued command, while nextStep's "
           "Times(1) above still holds, is the proof the Algorithm is never re-consulted during "
           "row-7 retries";
}

TEST_F(DroneControl, ScanHappensOnlyOnceAndOnlyAfterEventualMovementSuccess) {
    ON_CALL(gps_, position())
        .WillByDefault(Invoke(gpsPositionSequence(
            {Position3D{}, Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}})));

    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{false, "blocked"}));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).Times(1).WillOnce(Return(nonEmptyScanResult()));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, Row12ValidationOccursOnlyAfterSuccessfulMovementNotAfterFalseAttempts) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .WillOnce(Return(MovementResult{false, "blocked"}))
        .WillOnce(Return(MovementResult{false, "blocked again"}))
        .WillOnce(Return(MovementResult{true, ""}));

    // Exactly 2 gps_.position() reads total: the pre-step (row 11) read, and the post-movement
    // (row 12) read after the eventual success -- none at all for the two rejected false
    // attempts in between.
    EXPECT_CALL(gps_, position())
        .Times(2)
        .WillOnce(Return(Position3D{}))
        .WillOnce(Return(Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
    EXPECT_CALL(gps_, heading()).WillRepeatedly(Return(Orientation{}));

    const DroneStepResult result = control_.step();
    EXPECT_EQ(result.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, MidRow8ChunkRetriesSameChunkAndPreservesLaterChunks) {
    // droneConfig() max_advance=50cm; 150cm requested -> 3 chunks of 50cm. Chunk 2 fails once
    // before succeeding on retry -- the SAME 50cm chunk, dispatched again within the same
    // step() call -- and chunk 3 (the still-pending remainder) is then dispatched normally on
    // the next step() call, exactly as if chunk 2 had never failed.
    //
    // This test also calls control_.state() between step()s (each of which itself reads
    // gps_.position()), so a fixed pre-computed sequence of readings (as used elsewhere in this
    // file) would be thrown out of alignment by those extra calls. Instead, gps_.position()
    // always reports a `cumulative_x_cm` that a successful movement_.advance() action keeps up
    // to date -- correct regardless of how many extra times either mock gets called, and
    // correctly left unchanged by the rejected false attempt.
    auto cumulative_x_cm = std::make_shared<double>(0.0);
    ON_CALL(gps_, position()).WillByDefault(Invoke([cumulative_x_cm]() {
        return Position3D{*cumulative_x_cm * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    }));
    auto advanceAndSucceed = [cumulative_x_cm](PhysicalLength distance) {
        *cumulative_x_cm += distance.force_numerical_value_in(cm);
        return MovementResult{true, ""};
    };

    MappingStepCommand command;
    command.movement = advanceCommand(150.0 * isq::length[cm]);
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(1).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Invoke(advanceAndSucceed)); // chunk 1
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm]))
        .WillOnce(Return(MovementResult{false, "blocked"})); // chunk 2, attempt 1: no position change
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm]))
        .WillOnce(Invoke(advanceAndSucceed)); // chunk 2, attempt 2
    EXPECT_CALL(movement_, advance(50.0 * isq::length[cm])).WillOnce(Invoke(advanceAndSucceed)); // chunk 3

    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue); // chunk 1
    EXPECT_EQ(control_.state().step_index, 1u);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue)  // chunk 2 (retried once, still one step)
        << "the retry happens within this single step() call, not as an extra step";
    EXPECT_EQ(control_.state().step_index, 2u);
    EXPECT_EQ(control_.step().status, DroneStepStatus::Continue); // chunk 3, unaffected by chunk 2's retry
    EXPECT_EQ(control_.state().step_index, 3u);
}
