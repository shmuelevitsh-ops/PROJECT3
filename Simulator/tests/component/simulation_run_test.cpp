// Migrated from Project 2 (FILES PROJECT 2/tests/components/simulation_run_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Structurally adapted for Project 3's DI/module layout (see
// §3/§7.2): SimulationRunImpl's constructor drops the IDroneControl unique_ptr entirely
// (MissionControl now owns/builds its own drone control internally, per §3's "Each Mission
// Control provides and constructs its own drone-control implementation internally" rule) --
// every fixture/helper below that used to build and pass a GMockIDroneControl no longer
// does, and ConstructorThrowsInvalidArgumentIfAnyDependencyIsNull's null-checked dependency
// list shrinks from 8 to 7 accordingly. Map stubs, resolution-status helpers, and scoring
// assertions are otherwise unchanged, since MapsComparison, IMutableMap3D, and the
// constructor's remaining 7 parameters are structurally the same as Project 2 (§7.2).
// FactoryWritesOutputMapWithNpyExtensionAsAValidArray additionally needed a real IMissionControl
// double (SavingMissionControl below) instead of a bare NiceMock: unlike Project 2 (which linked
// the real MissionControlImpl), that test drives the real SimulationRunFactoryImpl with an
// injected factory (§3's dlopen-plugin architecture), and only the real MissionControlImpl's
// unconditional output_map_.save() call -- which a bare mock doesn't reproduce -- is what puts
// map_output.npy on disk for this test to find.

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/SimulationRunImpl.h>

#include "mocks/GMockIDroneMovement.h"
#include "mocks/GMockIGPS.h"
#include "mocks/GMockILidar.h"
#include "mocks/GMockIMappingAlgorithm.h"
#include "mocks/GMockIMissionControl.h"
#include "mocks/GMockIMutableMap3D.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#ifndef DATA_MAPS_DIR
#define DATA_MAPS_DIR "."
#endif

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

constexpr double kEpsilon = 1e-9;

Position3D origin() {
    return Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
}

Orientation headingDeg(double horizontal_deg, double altitude_deg) {
    return Orientation{horizontal_deg * horizontal_angle[deg], altitude_deg * altitude_angle[deg]};
}

// Reproduces the one MissionControlImpl behavior FactoryWritesOutputMapWithNpyExtensionAsAValidArray
// below actually depends on (unconditionally saving the output map before returning --
// MissionControlImpl.cpp) without running a real mission loop. A bare
// NiceMock<GMockIMissionControl> (uninteresting-call default) returns a default MissionRunResult
// but never touches output_map, so nothing ever gets written to disk for that test to find.
class SavingMissionControl final : public IMissionControl {
public:
    SavingMissionControl(IMutableMap3D& output_map, std::filesystem::path output_map_file)
        : output_map_(output_map), output_map_file_(std::move(output_map_file)) {}

    MissionRunResult runMission() override {
        output_map_.save(output_map_file_);
        return MissionRunResult{};
    }

private:
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
};

// ── SimulationRunImpl helpers ────────────────────────────────────────────────

MappingBounds boundsFromVoxelCount(long count, double resolution_cm) {
    const double extent_cm = static_cast<double>(count) * resolution_cm;
    return MappingBounds{
        0.0 * x_extent[cm], extent_cm * x_extent[cm],
        0.0 * y_extent[cm], extent_cm * y_extent[cm],
        0.0 * z_extent[cm], extent_cm * z_extent[cm]};
}

MapConfig gridConfig(long voxels_per_axis, double resolution_cm) {
    return MapConfig{boundsFromVoxelCount(voxels_per_axis, resolution_cm), Position3D{}, resolution_cm * isq::length[cm]};
}

std::unique_ptr<Map3DImpl> freshMap(const MapConfig& config) {
    return std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), config);
}

// A fixed-content IMutableMap3D stub: returns `config` from getMapConfig() and `fill` from every
// atVoxel() call. SimulationRunImpl::run() always calls the real MapsComparison::compare() (a
// hardwired collaborator, not injected through an interface) against hidden_map/output_map, so
// these tests cannot avoid exercising that real comparison logic. They should not, however, also
// depend on Map3DImpl's real allocation/clamping/indexing behavior (a separate component) just to
// supply that comparison with data. Stubbing both maps with identical fixed content keeps these
// tests isolated from Map3DImpl bugs while still running the real (and separately well-tested)
// "identical maps" path through MapsComparison::compare().
std::unique_ptr<NiceMock<test::GMockIMutableMap3D>> makeStubMap(const MapConfig& config,
                                                                VoxelOccupancy fill = VoxelOccupancy::Occupied) {
    auto map = std::make_unique<NiceMock<test::GMockIMutableMap3D>>();
    ON_CALL(*map, getMapConfig()).WillByDefault(Return(config));
    ON_CALL(*map, atVoxel(_)).WillByDefault(Return(fill));
    return map;
}

DroneConfigData droneConfig() {
    DroneConfigData config;
    config.radius = 5.0 * isq::length[cm];
    config.max_rotate = 45.0 * horizontal_angle[deg];
    config.max_advance = 50.0 * isq::length[cm];
    config.max_elevate = 40.0 * isq::length[cm];
    return config;
}

MissionConfigData missionConfig(double output_mapping_resolution_factor = 1.0) {
    MissionConfigData config;
    config.max_steps = 10;
    config.gps_resolution = 10.0 * isq::length[cm];
    config.output_mapping_resolution_factor = output_mapping_resolution_factor;
    return config;
}

LidarConfigData lidarConfig() {
    LidarConfigData config;
    config.z_min = 5.0 * isq::length[cm];
    config.z_max = 50.0 * isq::length[cm];
    config.d = 1.0 * isq::length[cm];
    config.fov_circles = 1;
    return config;
}

SimulationConfigData simulationConfig() {
    SimulationConfigData config;
    config.map_resolution = 10.0 * isq::length[cm];
    config.initial_drone_position = Position3D{};
    config.initial_angle = 0.0 * horizontal_angle[deg];
    return config;
}

// Builds a fully-wired SimulationRunImpl. hidden_map/output_map are stubbed
// IMutableMap3D doubles with identical fixed content (see makeStubMap()), not
// real Map3DImpl instances — this still exercises the real, hardwired
// MapsComparison::compare() call inside run(), but without depending on
// Map3DImpl's own (separately tested) behavior. The other four dependencies
// are NiceMock doubles, since run() never calls them directly — they only
// need to exist to satisfy ownership.
std::unique_ptr<SimulationRunImpl> makeSimulationRun(const MapConfig& hidden_config,
                                                     const MapConfig& output_config,
                                                     std::unique_ptr<IMissionControl> mission_control,
                                                     const MissionConfigData& mission_config = missionConfig(),
                                                     const std::filesystem::path& output_map_file = "out.npy") {
    auto hidden_map = makeStubMap(hidden_config);
    auto output_map = makeStubMap(output_config);

    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    auto movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
    auto lidar = std::make_unique<NiceMock<test::GMockILidar>>();
    // Reference *output_map before ownership moves below — the underlying
    // object's address is unaffected by the unique_ptr move, so this
    // reference stays valid for as long as SimulationRunImpl is alive.
    auto mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
        MappingAlgorithmDependencies{mission_config, lidarConfig(), droneConfig(), *output_map});

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulationConfig(),
        mission_config,
        output_map_file);
}

} // namespace

class SimulationRun : public ::testing::Test {
protected:
    // A fresh 20-voxel-per-axis (200cm), 10cm-resolution hidden map, defaulting to Unmapped
    // everywhere -- never Occupied, so it never blocks the plain (non-collision) movement tests
    // below. Individual collision tests mark specific voxels Occupied via hidden_map_->set(...).
    std::unique_ptr<Map3DImpl> hidden_map_ = freshMap(gridConfig(20, 10.0));
    MockGPS gps_{origin(), headingDeg(0.0, 0.0), 10.0 * isq::length[cm]};
    MockMovement movement_{gps_, *hidden_map_, droneConfig().radius};
};

// ── MockGPS ──────────────────────────────────────────────────────────────────

TEST_F(SimulationRun, GpsPositionRoundtrip) {
    const Position3D new_pos{12.0 * x_extent[cm], -7.0 * y_extent[cm], 3.0 * z_extent[cm]};
    gps_.setPosition(new_pos);

    EXPECT_EQ(gps_.position().x, new_pos.x);
    EXPECT_EQ(gps_.position().y, new_pos.y);
    EXPECT_EQ(gps_.position().z, new_pos.z);
}

TEST_F(SimulationRun, GpsHeadingRoundtrip) {
    const Orientation new_heading = headingDeg(90.0, 30.0);
    gps_.setHeading(new_heading);

    EXPECT_EQ(gps_.heading().horizontal, new_heading.horizontal);
    EXPECT_EQ(gps_.heading().altitude, new_heading.altitude);
}

// ── MockMovement::rotate ─────────────────────────────────────────────────────

TEST_F(SimulationRun, RotateLeftIncreasesHorizontalHeading) {
    const HorizontalAngle initial = gps_.heading().horizontal;

    const MovementResult result = movement_.rotate(RotationDirection::Left, 45.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result);
    EXPECT_EQ(gps_.heading().horizontal, initial + 45.0 * horizontal_angle[deg]);
}

TEST_F(SimulationRun, RotateRightDecreasesHorizontalHeading) {
    const HorizontalAngle initial = gps_.heading().horizontal;

    const MovementResult result = movement_.rotate(RotationDirection::Right, 45.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result);
    EXPECT_EQ(gps_.heading().horizontal, initial - 45.0 * horizontal_angle[deg]);
}

// ── MockMovement::advance ────────────────────────────────────────────────────

TEST_F(SimulationRun, AdvanceAtHeadingZeroMovesAlongX) {
    const MovementResult result = movement_.advance(50.0 * cm);

    EXPECT_TRUE(result);
    EXPECT_NEAR(gps_.position().x.force_numerical_value_in(cm), 50.0, kEpsilon);
    EXPECT_NEAR(gps_.position().y.force_numerical_value_in(cm), 0.0, kEpsilon);
    EXPECT_EQ(gps_.position().z, 0.0 * z_extent[cm]);
}

TEST_F(SimulationRun, AdvanceAtHeadingNinetyMovesAlongY) {
    gps_.setHeading(headingDeg(90.0, 0.0));

    const MovementResult result = movement_.advance(50.0 * cm);

    EXPECT_TRUE(result);
    EXPECT_NEAR(gps_.position().x.force_numerical_value_in(cm), 0.0, kEpsilon);
    EXPECT_NEAR(gps_.position().y.force_numerical_value_in(cm), 50.0, kEpsilon);
    EXPECT_EQ(gps_.position().z, 0.0 * z_extent[cm]);
}

TEST_F(SimulationRun, AdvanceAtHeadingFortyFiveMovesDiagonally) {
    gps_.setHeading(headingDeg(45.0, 0.0));

    const MovementResult result = movement_.advance(10.0 * cm);

    EXPECT_TRUE(result);
    const double expected_component = 10.0 * std::sqrt(2.0) / 2.0;
    EXPECT_NEAR(gps_.position().x.force_numerical_value_in(cm), expected_component, kEpsilon);
    EXPECT_NEAR(gps_.position().y.force_numerical_value_in(cm), expected_component, kEpsilon);
}

// ── MockMovement::elevate ────────────────────────────────────────────────────

TEST_F(SimulationRun, ElevatePositiveIncreasesAltitude) {
    const MovementResult result = movement_.elevate(20.0 * cm);

    EXPECT_TRUE(result);
    EXPECT_EQ(gps_.position().z, 20.0 * z_extent[cm]);
    EXPECT_EQ(gps_.position().x, 0.0 * x_extent[cm]);
    EXPECT_EQ(gps_.position().y, 0.0 * y_extent[cm]);
}

TEST_F(SimulationRun, ElevateNegativeDecreasesAltitude) {
    gps_.setPosition(Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]});

    const MovementResult result = movement_.elevate(-20.0 * cm);

    EXPECT_TRUE(result);
    EXPECT_EQ(gps_.position().z, 30.0 * z_extent[cm]);
}

// ── MockMovement collision detection (hidden-map wall checks) ───────────────

// Regression test for a lower-bound enumeration bug: sphereHitsWall() derived its minimum
// candidate voxel index as floor((center - radius) / resolution), which rounds *up* to the wrong
// voxel whenever (center - radius) lands exactly on a resolution boundary, silently skipping the
// voxel immediately below that boundary. Here the Occupied voxel spans x=[60,70), the drone
// center sits at x=75 with a 5cm radius, so center-radius=70 exactly -- the sphere's surface
// touches the voxel's far (x=70) face exactly, which sphereIntersectsAxisAlignedBox's closed
// (<=) comparison defines as intersecting. Uses a zero-distance advance() so the exact drone
// center is checked directly, isolating the point check from path sampling.
TEST_F(SimulationRun, AdvanceDetectsSphereExactlyTouchingVoxelFarFaceAtResolutionBoundary) {
    hidden_map_->set(Position3D{65.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);
    gps_.setPosition(Position3D{75.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    ASSERT_EQ(droneConfig().radius, 5.0 * isq::length[cm]);

    EXPECT_THROW(movement_.advance(0.0 * cm), SimulationException);
}

TEST_F(SimulationRun, AdvanceIntoOccupiedDestinationThrowsMovementCollision) {
    // Destination of advance(25cm) from the origin, heading 0deg (+x), is (25,0,0) -- inside
    // voxel (2,0,0) on this 10cm grid. Marking that voxel Occupied must block the move.
    hidden_map_->set(Position3D{25.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);

    try {
        movement_.advance(25.0 * cm);
        FAIL() << "expected a SimulationException(\"MOVEMENT_COLLISION\")";
    } catch (const SimulationException& e) {
        EXPECT_EQ(e.code(), "MOVEMENT_COLLISION");
    }
}

TEST_F(SimulationRun, WallOnlyInMiddleOfPathWithClearDestinationStillThrows) {
    // Destination (50,0,0) is left entirely clear (Unmapped); the wall sits only at x=25, well
    // inside the advance(50cm) path from the origin. The 0.1*resolution=1cm path sampling must
    // catch it even though the final destination itself is safe.
    hidden_map_->set(Position3D{25.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);

    EXPECT_THROW(movement_.advance(50.0 * cm), SimulationException);
}

TEST_F(SimulationRun, CenterlineClearButDroneRadiusOverlapsOccupiedVoxelThrows) {
    // The straight-line centerline (y=5,z=5) never itself enters an Occupied voxel, but a 6cm
    // drone radius reaches 1cm past the centerline's own voxel boundary into a neighboring
    // Occupied voxel offset in y -- collision must be detected from that sphere-vs-AABB radius
    // overlap, not only from whichever voxel each sampled point's own center falls inside.
    gps_.setPosition(Position3D{5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    hidden_map_->set(Position3D{15.0 * x_extent[cm], 15.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);
    MockMovement wide_movement(gps_, *hidden_map_, 6.0 * cm);

    EXPECT_THROW(wide_movement.advance(15.0 * cm), SimulationException);
}

TEST_F(SimulationRun, ClearAdvanceSucceedsAndReachesExactDestination) {
    const MovementResult result = movement_.advance(30.0 * cm);

    EXPECT_TRUE(result);
    EXPECT_NEAR(gps_.position().x.force_numerical_value_in(cm), 30.0, kEpsilon);
    EXPECT_NEAR(gps_.position().y.force_numerical_value_in(cm), 0.0, kEpsilon);
    EXPECT_NEAR(gps_.position().z.force_numerical_value_in(cm), 0.0, kEpsilon);
}

TEST_F(SimulationRun, CollisionLeavesGpsPositionUnchanged) {
    const Position3D before = gps_.position();
    hidden_map_->set(Position3D{25.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);

    EXPECT_THROW(movement_.advance(25.0 * cm), SimulationException);

    EXPECT_EQ(gps_.position().x, before.x);
    EXPECT_EQ(gps_.position().y, before.y);
    EXPECT_EQ(gps_.position().z, before.z);
}

TEST_F(SimulationRun, ElevateIntoOccupiedDestinationThrowsMovementCollisionAndLeavesGpsUnchanged) {
    const Position3D before = gps_.position();
    hidden_map_->set(Position3D{5.0 * x_extent[cm], 5.0 * y_extent[cm], 25.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);

    try {
        movement_.elevate(25.0 * cm);
        FAIL() << "expected a SimulationException(\"MOVEMENT_COLLISION\")";
    } catch (const SimulationException& e) {
        EXPECT_EQ(e.code(), "MOVEMENT_COLLISION");
    }

    EXPECT_EQ(gps_.position().x, before.x);
    EXPECT_EQ(gps_.position().y, before.y);
    EXPECT_EQ(gps_.position().z, before.z);
}

TEST_F(SimulationRun, NonMultipleOfSamplingStepDistanceStillChecksExactDestination) {
    // advance(24.3cm) is not a whole multiple of the 1cm (0.1*resolution) sampling step: the
    // regular samples land at x=0,1,...,24 (all >=6cm from the wall's near face at x=30), and
    // only the true fractional destination x=24.3 (5.7cm from x=30) is within this movement's
    // 5.8cm drone radius. A path check that skips the exact destination would miss this
    // collision entirely.
    hidden_map_->set(Position3D{35.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]},
                     VoxelOccupancy::Occupied);
    gps_.setPosition(Position3D{0.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    MockMovement precise_movement(gps_, *hidden_map_, 5.8 * cm);

    EXPECT_THROW(precise_movement.advance(24.3 * cm), SimulationException);
}

// ── SimulationRunImpl::run() ─────────────────────────────────────────────────

TEST_F(SimulationRun, RunReturnsMissionResultFromMissionControl) {
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    const MissionRunResult expected{MissionRunStatus::Completed, 7, {ErrorRef{"CODE", "message"}}};
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(expected));

    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control));
    const SimulationResult result = run->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, expected.status);
    EXPECT_EQ(result.mission_results[0].steps, expected.steps);
    ASSERT_EQ(result.mission_results[0].errors.size(), 1u);
    EXPECT_EQ(result.mission_results[0].errors[0].code, "CODE");
    EXPECT_EQ(result.mission_results[0].errors[0].message, "message");
}

// New — Project 3-only regression test (not a Project 2 port; SIM16 was explicitly excluded
// from Phase 1 for exactly this reason, see PROJ2_TESTS_EXECUTION_PLAN.md Phase 1 exception
// note / PROJ2_TESTS_PLAN.md §10). Protects the already-applied fix in SimulationRunImpl.cpp
// ("// fix from project 2": mission_result.status == Error -> mission_score = -1.0 instead of
// falling through to MapsComparison::compare()) against the carried-over SIM16 bug, where a
// mission that failed normally (MissionRunStatus::Error returned from mission_control_->
// runMission(), not thrown) was scored as if MapsComparison::compare() had run, and the
// resulting MissionRunResult was silently dropped from mission_results (SIM19's coverage gap --
// folded into this same test per §10.1, not duplicated as a separate one). Reuses
// RunReturnsMissionResultFromMissionControl's shape as its direct template (§10.1), with a
// normal-return Error status instead of Completed -- deliberately not the thrown-exception path,
// which RunPropagatesExceptionFromMissionControlRatherThanSwallowingIt already covers separately.
//
// Built via the manual/direct constructor pattern (as RunScoresFromTheRealMapsComparisonRatherThan
// AHardcodedValue does) rather than makeSimulationRun(), so the hidden_map/output_map mocks stay
// directly accessible for the atVoxel() expectations below -- confirmed (§10.2, by reading both
// SimulationRunImpl.cpp and MapsComparison.cpp) that the fix short-circuits entirely before
// calling MapsComparison::compare() on the error path, so atVoxel() -- the only call
// MapsComparison::compare() itself makes on either map, per MapsComparison.cpp -- must never be
// reached; getMapConfig() is deliberately not used for this since SimulationRunImpl::run() calls
// it unconditionally regardless of comparison, and so would prove nothing.
TEST_F(SimulationRun, ErrorStatusFromMissionControlScoresNegativeOnePreservesResultAndSkipsMapComparison) {
    auto hidden_map = makeStubMap(gridConfig(4, 10.0), VoxelOccupancy::Occupied);
    auto output_map = makeStubMap(gridConfig(4, 10.0), VoxelOccupancy::Occupied);
    EXPECT_CALL(*hidden_map, atVoxel(_)).Times(0);
    EXPECT_CALL(*output_map, atVoxel(_)).Times(0);

    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    auto movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
    auto lidar = std::make_unique<NiceMock<test::GMockILidar>>();
    auto mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
        MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *output_map});

    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    const MissionRunResult expected{
        MissionRunStatus::Error, 3, {ErrorRef{"DRONE_CONTROL_ERROR", "blocked by obstacle"}}};
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(expected));

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
                         std::move(lidar), std::move(mapping_algorithm),
                         std::move(mission_control), simulationConfig(), missionConfig(), "out.npy");
    const SimulationResult result = run.run();

    EXPECT_DOUBLE_EQ(result.mission_score, -1.0)
        << "an Error MissionRunResult returned normally (not thrown) from mission_control_ must "
           "score -1.0, not whatever MapsComparison::compare() would otherwise have produced";

    ASSERT_EQ(result.mission_results.size(), 1u)
        << "the errored MissionRunResult must still be preserved in mission_results, not dropped";
    EXPECT_EQ(result.mission_results[0].status, expected.status);
    EXPECT_EQ(result.mission_results[0].steps, expected.steps);
    ASSERT_EQ(result.mission_results[0].errors.size(), 1u);
    EXPECT_EQ(result.mission_results[0].errors[0].code, "DRONE_CONTROL_ERROR");
    EXPECT_EQ(result.mission_results[0].errors[0].message, "blocked by obstacle");
}

TEST_F(SimulationRun, RunPopulatesOutputMapFileFromConstructorArgument) {
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const std::filesystem::path expected_file = "output_results/run_42.npy";
    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control), missionConfig(), expected_file);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.output_map_file, expected_file);
}

TEST_F(SimulationRun, RunPopulatesOutputMapConfigFromOutputMap) {
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const MapConfig output_config = gridConfig(6, 10.0);
    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), output_config, std::move(mission_control));
    const SimulationResult result = run->run();

    EXPECT_EQ(result.output_map_config.resolution, output_config.resolution);
    EXPECT_EQ(result.output_map_config.boundaries.max_x, output_config.boundaries.max_x);
}

TEST_F(SimulationRun, RunPopulatesSimulationAndMissionConfigPassedToConstructor) {
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const MissionConfigData mission_config = missionConfig(2.0);
    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control), mission_config);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.mission_config.max_steps, mission_config.max_steps);
    EXPECT_EQ(result.mission_config.gps_resolution, mission_config.gps_resolution);
    EXPECT_EQ(result.simulation_config.map_resolution, simulationConfig().map_resolution);
}

TEST_F(SimulationRun, RunComputesNonNegativeScoreOnSuccess) {
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control));
    const SimulationResult result = run->run();

    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(SimulationRun, RunScoresFromTheRealMapsComparisonRatherThanAHardcodedValue) {
    // makeStubMap() defaults both hidden_map and output_map to identical (Occupied) content,
    // which would also pass a hardcoded "always return 100" bug. Override output_map's content to
    // Empty (fully mismatched against the Occupied hidden_map) and require the precise 0.0 that
    // only a real MapsComparison::compare() call would produce -- not just "some non-negative
    // number" -- so a bug that skips the real comparison (or always returns a fixed/positive
    // score) is caught.
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    auto hidden_map = makeStubMap(gridConfig(4, 10.0), VoxelOccupancy::Occupied);
    auto output_map = makeStubMap(gridConfig(4, 10.0), VoxelOccupancy::Empty);
    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    auto movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
    auto lidar = std::make_unique<NiceMock<test::GMockILidar>>();
    auto mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
        MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *output_map});

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
                         std::move(lidar), std::move(mapping_algorithm),
                         std::move(mission_control), simulationConfig(), missionConfig(), "out.npy");
    const SimulationResult result = run.run();

    EXPECT_DOUBLE_EQ(result.mission_score, 0.0);
}

TEST_F(SimulationRun, RunScoresHiddenMapAsGroundTruthNotAsCandidate) {
    // RunScoresFromTheRealMapsComparisonRatherThanAHardcodedValue above uses a fully-Occupied vs.
    // fully-Empty mismatch, which is symmetric under argument order (swapping which map is
    // "origin" vs. "target" still yields a complete mismatch, score 0.0, either way) -- it cannot
    // distinguish MapsComparison::compare(*hidden_map_, {output_map_.get()}) from the swapped
    // MapsComparison::compare(*output_map_, {hidden_map_.get()}). This test uses an asymmetric
    // case that only the correct argument order scores perfectly: the hidden (ground-truth) map
    // has one Unmapped voxel (skipped when used as *origin*, since "no ground truth" must not
    // penalize the score) while the output map is fully Occupied everywhere, including at that
    // same position. With the correct order, the Unmapped voxel is skipped and every other voxel
    // matches -> 100.0. With the arguments swapped, the fully-Occupied output map (now origin) has
    // no Unmapped voxels to skip, so the hidden map's Unmapped voxel (now target) counts as a
    // mismatch -> a score below 100.0.
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const MapConfig config = gridConfig(2, 10.0); // 2x2x2 grid, zero offset, 10cm resolution.
    const Position3D unmapped_voxel = origin(); // one of the 8 sampled grid points.

    auto hidden_map = std::make_unique<NiceMock<test::GMockIMutableMap3D>>();
    ON_CALL(*hidden_map, getMapConfig()).WillByDefault(Return(config));
    ON_CALL(*hidden_map, atVoxel(_))
        .WillByDefault(testing::Invoke([unmapped_voxel](const Position3D& p) {
            return (p.x == unmapped_voxel.x && p.y == unmapped_voxel.y && p.z == unmapped_voxel.z)
                       ? VoxelOccupancy::Unmapped
                       : VoxelOccupancy::Occupied;
        }));

    auto output_map = makeStubMap(config, VoxelOccupancy::Occupied);
    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    auto movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
    auto lidar = std::make_unique<NiceMock<test::GMockILidar>>();
    auto mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
        MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *output_map});

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
                         std::move(lidar), std::move(mapping_algorithm),
                         std::move(mission_control), simulationConfig(), missionConfig(), "out.npy");
    const SimulationResult result = run.run();

    EXPECT_DOUBLE_EQ(result.mission_score, 100.0)
        << "the hidden map must be the comparison's origin (ground truth): its one Unmapped voxel "
           "must be skipped, not counted as a mismatch, so a fully-Occupied output map scores 100";
}

TEST_F(SimulationRun, RunScoresOutputMapAfterMissionControlPopulatesItNotBeforehand) {
    // All other scoring tests use a static (GMock-stubbed) output_map_ whose content never
    // changes regardless of when MapsComparison::compare() is called, so they cannot distinguish
    // scoring before vs. after mission_control_->runMission() runs. This test uses a real,
    // mutable Map3DImpl for output_map_ (starting entirely Unmapped) and a mocked
    // mission_control_ whose runMission() populates it to fully match the hidden map as a side
    // effect -- exactly mirroring how the real MissionControlImpl/DroneControlImpl pipeline
    // populates output_map_ via scanning. With the correct ordering (score after
    // runMission()), the now-fully-matching output map scores 100; if scoring happened before
    // runMission() ran, it would see the still-entirely-Unmapped map and score 0.
    auto hidden_map = makeStubMap(gridConfig(2, 10.0), VoxelOccupancy::Occupied);

    auto output_map = freshMap(gridConfig(2, 10.0)); // starts entirely Unmapped.
    Map3DImpl* output_map_ptr = output_map.get(); // reference stays valid across the move below.

    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(testing::Invoke([output_map_ptr]() {
        for (long ix = 0; ix < 2; ++ix) {
            for (long iy = 0; iy < 2; ++iy) {
                for (long iz = 0; iz < 2; ++iz) {
                    output_map_ptr->set(
                        Position3D{static_cast<double>(ix) * 10.0 * x_extent[cm],
                                   static_cast<double>(iy) * 10.0 * y_extent[cm],
                                   static_cast<double>(iz) * 10.0 * z_extent[cm]},
                        VoxelOccupancy::Occupied);
                }
            }
        }
        return MissionRunResult{};
    }));

    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    auto movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
    auto lidar = std::make_unique<NiceMock<test::GMockILidar>>();
    auto mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
        MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *output_map});

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
                         std::move(lidar), std::move(mapping_algorithm),
                         std::move(mission_control), simulationConfig(), missionConfig(), "out.npy");
    const SimulationResult result = run.run();

    EXPECT_DOUBLE_EQ(result.mission_score, 100.0)
        << "the score must reflect output_map_'s state *after* runMission() populates it, not its "
           "still-Unmapped state before the mission ran";
}

TEST_F(SimulationRun, RunPropagatesExceptionFromMissionControlRatherThanSwallowingIt) {
    // SimulationManager's documented "score -1 and continue" error-handling contract depends on
    // exceptions from inside a run propagating all the way out of run() uncaught. A bug that adds
    // a local try/catch here (even a well-intentioned one) would silently swallow the failure and
    // break that contract.
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    EXPECT_CALL(*mission_control, runMission()).WillOnce([]() -> MissionRunResult {
        throw std::runtime_error("drone control wedged");
    });

    const std::unique_ptr<SimulationRunImpl> run =
        makeSimulationRun(gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control));

    EXPECT_THROW(run->run(), std::runtime_error);
}

TEST_F(SimulationRun, ResolutionRequestStatusAtExactlyFactorOneIsNotIgnoredTooSmall) {
    // output_mapping_resolution_factor's IgnoredTooSmall branch is guarded by `< 1.0`; exactly 1.0
    // protects the default factor from being misclassified as "too small".
    const MissionConfigData mission_config = missionConfig(1.0);
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const std::unique_ptr<SimulationRunImpl> run = makeSimulationRun(
        gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control), mission_config);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted)
        << "factor exactly 1.0 must not be treated as IgnoredTooSmall";
}

TEST_F(SimulationRun, ResolutionRequestStatusIgnoredWhenRequestedResolutionDiffersFromHiddenMapResolution) {
    // simulationConfig() (this file's shared helper) fixes simulation_config.map_resolution at
    // 10cm regardless of what MapConfig is manually fed to the stub output map below -- the
    // status must compare the mission's requested resolution (gps_resolution * factor) against
    // that hidden-map resolution, not against the stub output map's own reported MapConfig. A
    // requested 20cm (factor=2 on a 10cm gps_resolution) does not match the 10cm hidden map, so
    // this must be IGNORED, never a false ACCEPTED that could still end up scored -1.
    const MissionConfigData mission_config = missionConfig(2.0); // gps_resolution=10cm -> requested 20cm
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const std::unique_ptr<SimulationRunImpl> run = makeSimulationRun(
        gridConfig(4, 10.0), gridConfig(4, 20.0), std::move(mission_control), mission_config);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Ignored);
}

TEST_F(SimulationRun, OutputMapResolutionAlwaysReturnsSimulationMapResolutionRegardlessOfFactor) {
    // outputMapResolution() (SimulationRunFactoryImpl.cpp) is the function the factory actually
    // uses to build the output map. It must always return simulation_config.map_resolution -- the
    // hidden map's own resolution, and the only value MapsComparison can score against -- never
    // gps_resolution and never a mission's requested gps_resolution * factor (it does not even
    // take a MissionConfigData). map_resolution is deliberately set to a value no gps_resolution *
    // factor combination used elsewhere in this file would produce, so a bug that pulls the
    // resolution from the wrong config is unambiguously observable.
    SimulationConfigData simulation_config;
    simulation_config.map_resolution = 20.0 * isq::length[cm];

    const PhysicalLength resolution = outputMapResolution(simulation_config);

    EXPECT_EQ(resolution, 20.0 * isq::length[cm])
        << "outputMapResolution() must return simulation_config.map_resolution, not "
           "gps_resolution and not gps_resolution * factor";
}

TEST_F(SimulationRun, ResolutionRequestStatusIgnoredTooSmallWhenFactorBelowOne) {
    const MissionConfigData mission_config = missionConfig(0.5);
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    const std::unique_ptr<SimulationRunImpl> run = makeSimulationRun(
        gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control), mission_config);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::IgnoredTooSmall);
}

TEST_F(SimulationRun, ResolutionRequestStatusIgnoredWhenOutputResolutionDiffersFromRequested) {
    const MissionConfigData mission_config = missionConfig(2.0); // requested 20cm
    auto mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(MissionRunResult{}));

    // simulationConfig()'s hidden map resolution is fixed at 10cm, not the requested 20cm.
    const std::unique_ptr<SimulationRunImpl> run = makeSimulationRun(
        gridConfig(4, 10.0), gridConfig(4, 10.0), std::move(mission_control), mission_config);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Ignored);
}

// ── Factory-built output map config (SimulationRunFactoryImpl::outputMapConfig) ─────────────────

TEST_F(SimulationRun, FactoryBuiltOutputMapOffsetComesFromMissionBoundsNotHiddenMap) {
    // The mapping algorithm must stay unaware of the hidden map's geometry: output_map's offset
    // must be derived from mission.mission_bounds' minimum corner, never from the hidden
    // (ground-truth) map's own offset. Uses the real SimulationRunFactoryImpl (not a
    // SimulationRunImpl constructed directly with manual configs) since that's the only place
    // this wiring actually happens. Points at a real (tiny) .npy fixture, since a load failure
    // now fails the run instead of falling back to an empty map; max_steps=0 means the mission
    // loop never touches the real mapping algorithm, isolating this test to exactly the
    // map-construction wiring under test.
    //
    // The hidden map's deliberately-distant offset below (chosen so a bug that conflates the
    // output map's offset with the hidden map's offset would be unambiguously observable) also
    // means mission_bounds has no overlap with the hidden map's own bounds -- which, in current
    // Project 3 SimulationRunImpl::run(), now trips a newer boundary-validity guard added after
    // this test's original Project 2 design (returns an early MISSION_BOUNDARY_INVALID result
    // instead of running the normal mission-control/scoring path; see SimulationRunImpl.cpp).
    // That does not affect this test: output_map_'s MapConfig is set once during construction (by
    // the factory, before run() is ever called), and both of run()'s return paths copy it into
    // the result identically, so the offset assertions below hold regardless of which path
    // executes. Flagged here rather than silently adjusted, per PROJ2_TESTS_EXECUTION_PLAN.md
    // Phase 4's instruction to document rather than conceal a production-behavior discrepancy
    // exposed by a migrated test.
    SimulationConfigData simulation_config;
    simulation_config.map_filename = std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy";
    simulation_config.map_resolution = 10.0 * isq::length[cm];
    // Hidden map offset deliberately far from mission_bounds' minimum corner below, so a bug that
    // conflates the two is unambiguously observable.
    simulation_config.map_offset =
        Position3D{100.0 * x_extent[cm], 200.0 * y_extent[cm], 300.0 * z_extent[cm]};
    simulation_config.initial_drone_position = Position3D{};
    simulation_config.initial_angle = 0.0 * horizontal_angle[deg];

    MissionConfigData mission_config = missionConfig();
    mission_config.max_steps = 0;
    mission_config.mission_bounds = MappingBounds{
        0.0 * x_extent[cm], 10.0 * x_extent[cm],
        0.0 * y_extent[cm], 10.0 * y_extent[cm],
        0.0 * z_extent[cm], 10.0 * z_extent[cm]};

    const std::filesystem::path output_path =
        "tests/component/test_output/simulation_run_test/factory_output_map_offset";
    std::filesystem::remove_all(output_path);

    SimulationRunFactoryImpl factory(
        [](MappingAlgorithmDependencies dependencies) -> std::unique_ptr<IMappingAlgorithm> {
            return std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(std::move(dependencies));
        },
        [](MissionControlDependencies) -> std::unique_ptr<IMissionControl> {
            return std::make_unique<NiceMock<test::GMockIMissionControl>>();
        },
        /*verbose=*/false);
    const std::unique_ptr<ISimulationRun> run =
        factory.create(simulation_config, mission_config, droneConfig(), lidarConfig(), output_path);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.output_map_config.offset.x, mission_config.mission_bounds.min_x)
        << "output map's offset.x must come from mission_bounds, not the hidden map's own offset";
    EXPECT_EQ(result.output_map_config.offset.y, mission_config.mission_bounds.min_y);
    EXPECT_EQ(result.output_map_config.offset.z, mission_config.mission_bounds.min_height);
}

TEST_F(SimulationRun, FactoryWritesOutputMapWithNpyExtensionAsAValidArray) {
    // The factory's fixed per-run filename (kOutputMapFileName) must keep the documented .npy
    // extension: a bug that drops it would still produce a real, loadable file (only the name is
    // wrong), so this must check the actual path/extension, not just "a file got written".
    SimulationConfigData simulation_config;
    simulation_config.map_filename = std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy";
    simulation_config.map_resolution = 10.0 * isq::length[cm];
    simulation_config.initial_drone_position = Position3D{};
    simulation_config.initial_angle = 0.0 * horizontal_angle[deg];

    MissionConfigData mission_config = missionConfig();
    mission_config.max_steps = 0;
    mission_config.mission_bounds = MappingBounds{
        0.0 * x_extent[cm], 10.0 * x_extent[cm],
        0.0 * y_extent[cm], 10.0 * y_extent[cm],
        0.0 * z_extent[cm], 10.0 * z_extent[cm]};

    const std::filesystem::path output_path = "tests/component/test_output/simulation_run_test/factory_output_map_extension";
    std::filesystem::remove_all(output_path);

    SimulationRunFactoryImpl factory(
        [](MappingAlgorithmDependencies dependencies) -> std::unique_ptr<IMappingAlgorithm> {
            return std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(std::move(dependencies));
        },
        [](MissionControlDependencies dependencies) -> std::unique_ptr<IMissionControl> {
            return std::make_unique<SavingMissionControl>(dependencies.output_map, dependencies.output_map_file);
        },
        /*verbose=*/false);
    const std::unique_ptr<ISimulationRun> run =
        factory.create(simulation_config, mission_config, droneConfig(), lidarConfig(), output_path);
    const SimulationResult result = run->run();

    EXPECT_EQ(result.output_map_file.extension(), ".npy")
        << "the generated output map file must keep the documented .npy extension, got: "
        << result.output_map_file;
    ASSERT_TRUE(std::filesystem::exists(result.output_map_file));

    auto reloaded = std::make_shared<NpyArray>();
    const char* load_error = reloaded->LoadNPY(result.output_map_file.string());
    EXPECT_EQ(load_error, nullptr)
        << "the generated output map file must be loadable as a valid .npy array, got error: "
        << (load_error != nullptr ? load_error : "");
}

TEST_F(SimulationRun, FactoryBuiltOutputMapResolutionMatchesHiddenMapResolutionAndReportsStatusAccordingly) {
    // OutputMapResolutionAlwaysReturnsSimulationMapResolutionRegardlessOfFactor above calls
    // outputMapResolution() directly, but never checks that its result is actually *used* to
    // construct the output map -- a "two sources of truth" bug could compute the right value for
    // reporting purposes and then separately discard it, building the real output map at a
    // different resolution instead. This exercises the real factory end-to-end and checks the
    // actual constructed output map's resolution and status, not just the standalone formula, for
    // both a mismatched and a matching requested resolution.
    const std::filesystem::path map_filename = std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy";
    const MappingBounds mission_bounds{
        0.0 * x_extent[cm], 30.0 * x_extent[cm],
        0.0 * y_extent[cm], 30.0 * y_extent[cm],
        0.0 * z_extent[cm], 30.0 * z_extent[cm]};

    auto makeFactory = []() {
        return SimulationRunFactoryImpl(
            [](MappingAlgorithmDependencies dependencies) -> std::unique_ptr<IMappingAlgorithm> {
                return std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(std::move(dependencies));
            },
            [](MissionControlDependencies) -> std::unique_ptr<IMissionControl> {
                return std::make_unique<NiceMock<test::GMockIMissionControl>>();
            },
            /*verbose=*/false);
    };

    // Requested (gps=10cm * factor=3 = 30cm) does not match the hidden map's own 10cm resolution:
    // not supported end-to-end, so the output map falls back to the hidden map's 10cm and the
    // status is IGNORED.
    {
        SimulationConfigData simulation_config;
        simulation_config.map_filename = map_filename;
        simulation_config.map_resolution = 10.0 * isq::length[cm];
        simulation_config.initial_drone_position = Position3D{};
        simulation_config.initial_angle = 0.0 * horizontal_angle[deg];

        MissionConfigData mission_config = missionConfig(3.0); // requested 30cm; hidden map is 10cm
        mission_config.max_steps = 0;
        mission_config.mission_bounds = mission_bounds;

        const std::filesystem::path output_path = "tests/component/test_output/simulation_run_test/factory_output_map_resolution_ignored";
        std::filesystem::remove_all(output_path);

        SimulationRunFactoryImpl factory = makeFactory();
        const std::unique_ptr<ISimulationRun> run =
            factory.create(simulation_config, mission_config, droneConfig(), lidarConfig(), output_path);
        const SimulationResult result = run->run();

        EXPECT_EQ(result.output_map_config.resolution, 10.0 * isq::length[cm])
            << "requested resolution unsupported: output map must fall back to the hidden map's "
               "own resolution (10cm), not gps_resolution alone and not the unsupported requested "
               "30cm";
        EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Ignored)
            << "a requested resolution that does not match the hidden map's resolution must be "
               "reported as IGNORED, not ACCEPTED";
    }

    // Requested (gps=10cm * factor=2 = 20cm) matches the hidden map's own 20cm resolution: this
    // is genuinely supported end-to-end, so the output map is built at 20cm and the status is
    // ACCEPTED -- this is the concrete scenario from the assignment/forum clarification.
    {
        SimulationConfigData simulation_config;
        simulation_config.map_filename = map_filename;
        simulation_config.map_resolution = 20.0 * isq::length[cm];
        simulation_config.initial_drone_position = Position3D{};
        simulation_config.initial_angle = 0.0 * horizontal_angle[deg];

        MissionConfigData mission_config = missionConfig(2.0); // requested 20cm; hidden map is 20cm
        mission_config.max_steps = 0;
        mission_config.mission_bounds = mission_bounds;

        const std::filesystem::path output_path = "tests/component/test_output/simulation_run_test/factory_output_map_resolution_accepted";
        std::filesystem::remove_all(output_path);

        SimulationRunFactoryImpl factory = makeFactory();
        const std::unique_ptr<ISimulationRun> run =
            factory.create(simulation_config, mission_config, droneConfig(), lidarConfig(), output_path);
        const SimulationResult result = run->run();

        EXPECT_EQ(result.output_map_config.resolution, 20.0 * isq::length[cm])
            << "requested resolution matches the hidden map's resolution: output map must be "
               "built at 20cm, not gps_resolution (10cm)";
        EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted)
            << "a requested resolution that matches the hidden map's resolution is genuinely "
               "supported end-to-end and must be reported as ACCEPTED";
    }
}

TEST_F(SimulationRun, ConstructorThrowsInvalidArgumentIfAnyDependencyIsNull) {
    constexpr const char* kDependencyNames[] = {"hidden_map",      "output_map",   "gps",
                                                "movement",        "lidar",        "mapping_algorithm",
                                                "mission_control"};
    for (int null_index = 0; null_index < 7; ++null_index) {
        SCOPED_TRACE(std::string("nulled dependency: ") + kDependencyNames[null_index]);
        std::unique_ptr<const IMap3D> hidden_map_arg = freshMap(gridConfig(4, 10.0));
        std::unique_ptr<IMutableMap3D> output_map_arg = freshMap(gridConfig(4, 10.0));
        std::unique_ptr<IGPS> gps = std::make_unique<NiceMock<test::GMockIGPS>>();
        std::unique_ptr<IDroneMovement> movement = std::make_unique<NiceMock<test::GMockIDroneMovement>>();
        std::unique_ptr<ILidar> lidar = std::make_unique<NiceMock<test::GMockILidar>>();
        // Backed by its own map, independent of hidden_map_arg/output_map_arg,
        // so nulling either of those above never leaves a dangling reference here.
        auto algorithm_map = freshMap(gridConfig(4, 10.0));
        std::unique_ptr<IMappingAlgorithm> mapping_algorithm = std::make_unique<NiceMock<test::GMockIMappingAlgorithm>>(
            MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *algorithm_map});
        std::unique_ptr<IMissionControl> mission_control = std::make_unique<NiceMock<test::GMockIMissionControl>>();

        switch (null_index) {
            case 0: hidden_map_arg.reset(); break;
            case 1: output_map_arg.reset(); break;
            case 2: gps.reset(); break;
            case 3: movement.reset(); break;
            case 4: lidar.reset(); break;
            case 5: mapping_algorithm.reset(); break;
            case 6: mission_control.reset(); break;
        }

        EXPECT_THROW(
            (SimulationRunImpl(
                std::move(hidden_map_arg), std::move(output_map_arg), std::move(gps), std::move(movement),
                std::move(lidar), std::move(mapping_algorithm), std::move(mission_control),
                simulationConfig(), missionConfig(), "out.npy")),
            std::invalid_argument);
    }
}
