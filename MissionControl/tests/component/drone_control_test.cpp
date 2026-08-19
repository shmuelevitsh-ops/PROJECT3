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

#include <stdexcept>

using namespace common;
using namespace common::types;
using namespace MissionControl_322889890_315113738;
using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
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
};

} // namespace

TEST_F(DroneControl, FirstStepPassesNullLatestScanToAlgorithm) {
    EXPECT_CALL(algorithm_, nextStep(_, IsNull())).WillOnce(Return(MappingStepCommand{}));

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
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{}));

    (void)control_.step();
}

TEST_F(DroneControl, AdvanceMovementCallsAdvanceBeforeScan) {
    MappingStepCommand command;
    command.movement = advanceCommand(20.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, advance(20.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{}));

    (void)control_.step();
}

TEST_F(DroneControl, ElevateMovementCallsElevateBeforeScan) {
    MappingStepCommand command;
    command.movement = elevateCommand(15.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, elevate(15.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{}));

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
    MappingStepCommand command;
    command.movement = elevateCommand(-10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    InSequence seq;
    EXPECT_CALL(movement_, elevate(-10.0 * isq::length[cm])).WillOnce(Return(MovementResult{true, ""}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{}));

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
    MappingStepCommand command; // movement and scan_orientation both unset
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(_)).Times(0);

    (void)control_.step();
}

TEST_F(DroneControl, ScanOrientationSetCallsLidarScanWithGivenOrientation) {
    const Orientation orientation{30.0 * horizontal_angle[deg], 5.0 * altitude_angle[deg]};
    MappingStepCommand command;
    command.scan_orientation = orientation;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(lidar_, scan(OrientationEq(orientation))).WillOnce(Return(LidarScanResult{}));

    (void)control_.step();
}

TEST_F(DroneControl, ScanResultIsPassedAsLatestScanToNextStepOnly) {
    LidarHit hit;
    hit.distance = 42.0 * isq::length[cm];
    const LidarScanResult scan_result{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    MappingStepCommand idle_command; // no movement, no scan

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
    MappingStepCommand command;
    command.status = AlgorithmStatus::Working;
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

TEST_F(DroneControl, MovementFailureReturnsErrorWithoutScanningOrAdvancingStepIndex) {
    MappingStepCommand command;
    command.movement = advanceCommand(10.0 * isq::length[cm]);
    command.scan_orientation = Orientation{}; // should never be reached
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));

    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm]))
        .WillOnce(Return(MovementResult{false, "blocked by obstacle"}));
    EXPECT_CALL(lidar_, scan(_)).Times(0);

    const DroneStepResult result = control_.step();

    EXPECT_EQ(result.status, DroneStepStatus::Error);
    EXPECT_EQ(control_.state().step_index, 0u);
}

TEST_F(DroneControl, StepIndexIncrementsOnSuccessfulStepsOnly) {
    MappingStepCommand command; // no movement, no scan, Working
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
    MappingStepCommand command; // no movement, no scan, Working
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
    // scanned whenever an exception escapes from the scan block.
    MappingStepCommand command;
    command.scan_orientation = Orientation{};
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce([]() -> LidarScanResult { throw std::runtime_error("lidar fault"); });

    ASSERT_EQ(control_.state().step_index, 0u);
    EXPECT_THROW(control_.step(), std::runtime_error);
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
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(LidarScanResult{}));

    EXPECT_CALL(gps_, position()).Times(2).WillRepeatedly(Return(Position3D{}));
    EXPECT_CALL(gps_, heading()).Times(2).WillRepeatedly(Return(Orientation{}));

    (void)control_.step();
}

TEST_F(DroneControl, ScanResultAppliedAtPostMovementPositionNotPreMovement) {
    // A narrower variant of the bug above that targets only the *value* passed as the scan
    // origin into ScanResultToVoxels::applyToMap, not whether gps_.position() gets called again
    // (a bug could still call gps_.position() twice -- satisfying the call-count check above --
    // while wrongly discarding the second result and passing the first, pre-movement, value into
    // applyToMap instead). Verified by checking *where* a resulting Occupied voxel actually lands.
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

    (void)control_.step();
}

TEST_F(DroneControl, MovementFailureDoesNotClearPreviousLatestScan) {
    // Documented in DroneControlImpl::step(): on movement failure, "do not touch latest_scan_".
    // A bug that unconditionally resets it to nullopt (matching the no-scan-this-step branch)
    // would lose the previous step's scan data instead of preserving it.
    LidarHit hit;
    hit.distance = 10.0 * isq::length[cm];
    const LidarScanResult scan_result{hit};

    MappingStepCommand scanning_command;
    scanning_command.scan_orientation = Orientation{};

    MappingStepCommand failing_move_command;
    failing_move_command.movement = advanceCommand(10.0 * isq::length[cm]);

    MappingStepCommand idle_command;

    auto isNullScan = [](const LidarScanResult* scan) { return scan == nullptr; };
    auto isSingleHitScan = [](const LidarScanResult* scan) { return scan != nullptr && scan->size() == 1; };

    InSequence seq;
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isNullScan))).WillOnce(Return(scanning_command));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(scan_result));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(failing_move_command));
    EXPECT_CALL(movement_, advance(10.0 * isq::length[cm])).WillOnce(Return(MovementResult{false, "blocked"}));
    EXPECT_CALL(algorithm_, nextStep(_, Truly(isSingleHitScan))).WillOnce(Return(idle_command));

    (void)control_.step();                                  // step 1: scans, stores latest_scan_ (1 hit)
    const DroneStepResult failed = control_.step();          // step 2: movement fails before touching latest_scan_
    ASSERT_EQ(failed.status, DroneStepStatus::Error);
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
