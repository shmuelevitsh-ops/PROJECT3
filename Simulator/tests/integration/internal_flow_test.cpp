// Migrated from Project 2 (FILES PROJECT 2/tests/internal/internal_flow_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Confirmed baseline: 12/12 pass (PROJ2_TESTS_PLAN.md
// §5.1) -- near-1:1, no scenario/assertion changes. Adapted for Project 3's
// DI/module layout (see §3/§7.2): namespaces/includes across three modules
// (MissionControl, Algorithm, Simulator), plus the same MissionControlImpl DI
// shape change already applied to mission_control_test.cpp -- Project 2 built a
// DroneControlImpl by hand and passed it into MissionControlImpl; Project 3's
// MissionControlImpl builds its own DroneControlImpl internally from
// common::MissionControlDependencies, so every test below that used to do this
// now passes the same lidar/gps/movement/mapping_algorithm dependencies directly
// to MissionControlImpl instead (RealAlgorithmFlowOnSmallKnownMapDoesNotCrash
// similarly supplies real MappingAlgorithmFactory/MissionControlFactory
// callbacks to SimulationRunFactoryImpl, whose constructor now takes injected
// factories rather than statically linking one algorithm/mission-control pair —
// same DI shape already applied in simulation_run_factory_impl_test.cpp). No
// other behavioral changes from the Project 2 original.

// Phase 6 — Internal & Regression Tests, suite name `Internal` (deliberately
// NOT matched by the assignment's required `--gtest_filter=Integration.*`).
//
// Physically separated from tests/integration/full_flow_test.cpp so this file
// can be excluded from the final submission archive without touching
// anything the staff runs. Synthetic/trivial fixtures kept here for the
// author's own fast iteration and edge-case hunting: an entrance-size sweep
// on a hand-built two-room box, a 5x5x5 single-voxel sanity check, and a
// Hover-only MaxSteps edge case.

#include <Algorithm/MappingAlgorithmImpl.h>
#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/MissionControlImpl.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include "mocks/GMockILidar.h"
#include "mocks/GMockIMappingAlgorithm.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;
using namespace MissionControl_322889890_315113738;
using namespace Algorithm_322889890_315113738;

namespace {

std::filesystem::path scratchDir() {
    // Relative to CWD, not an absolute path -- same convention (and same
    // output directory) as tests/integration/full_flow_test.cpp's
    // scratchDir(), per explicit instruction. See .gitignore for why it's
    // never committed.
    const std::filesystem::path dir = std::filesystem::path("tests/integration/test_output");
    std::filesystem::create_directories(dir);
    return dir;
}

// ── Synthetic two-room map: Room A | dividing wall (with one opening) | Room B ──

// Builds a solid-shell box of size nx*ny*nz voxels, fully Empty inside, with a
// dividing wall at the x midpoint separating "Room A" (x < wall_x) from "Room
// B" (x > wall_x), pierced by one opening_w x opening_h opening centered in
// the wall's interior (y, z) extent.
std::shared_ptr<NpyArray> buildTwoRoomMap(long nx, long ny, long nz, long opening_w, long opening_h) {
    auto array = std::make_shared<NpyArray>(
        NpyArray::shape_t{static_cast<size_t>(nx), static_cast<size_t>(ny), static_cast<size_t>(nz)},
        sizeof(int), NpyArray::GetTypeChar(typeid(int)));
    array->Allocate();
    int* data = array->Data<int>();
    const auto index = [ny, nz](long x, long y, long z) { return x * ny * nz + y * nz + z; };

    std::fill_n(data, array->NumValue(), 0); // Empty everywhere by default

    for (long y = 0; y < ny; ++y) {
        for (long z = 0; z < nz; ++z) {
            data[index(0, y, z)] = 1;
            data[index(nx - 1, y, z)] = 1;
        }
    }
    for (long x = 0; x < nx; ++x) {
        for (long z = 0; z < nz; ++z) {
            data[index(x, 0, z)] = 1;
            data[index(x, ny - 1, z)] = 1;
        }
    }
    for (long x = 0; x < nx; ++x) {
        for (long y = 0; y < ny; ++y) {
            data[index(x, y, 0)] = 1;
            data[index(x, y, nz - 1)] = 1;
        }
    }

    const long wall_x = nx / 2;
    for (long y = 0; y < ny; ++y) {
        for (long z = 0; z < nz; ++z) {
            data[index(wall_x, y, z)] = 1;
        }
    }

    const long interior_y0 = 1;
    const long interior_z0 = 1;
    const long interior_h_y = ny - 2;
    const long interior_h_z = nz - 2;
    const long y_start = interior_y0 + (interior_h_y - opening_w) / 2;
    const long z_start = interior_z0 + (interior_h_z - opening_h) / 2;
    for (long dy = 0; dy < opening_w; ++dy) {
        for (long dz = 0; dz < opening_h; ++dz) {
            data[index(wall_x, y_start + dy, z_start + dz)] = 0;
        }
    }

    return array;
}

// Room B is made deep enough (relative to lidar z_max = 50cm = 5 voxels)
// that its far wall cannot be seen by a beam fired through the opening from
// Room A — only a drone that physically entered Room B can ever observe it.
// Without this margin, a lidar beam can see *through* a hole regardless of
// whether the drone's own safety sphere could fit through it, since entrance
// size constrains movement, not visibility — confirmed by the first version
// of this test, which had Room A/B only 3 voxels deep and reported
// "Room B observed" for every drone size, fitting or not.
constexpr long kRoomNx = 19;
constexpr long kRoomNy = 8;
constexpr long kRoomNz = 8;
constexpr double kRoomResolutionCm = 10.0;

MapConfig roomMapConfig() {
    return MapConfig{
        MappingBounds{
            0.0 * x_extent[cm], static_cast<double>(kRoomNx) * kRoomResolutionCm * x_extent[cm],
            0.0 * y_extent[cm], static_cast<double>(kRoomNy) * kRoomResolutionCm * y_extent[cm],
            0.0 * z_extent[cm], static_cast<double>(kRoomNz) * kRoomResolutionCm * z_extent[cm]},
        Position3D{},
        kRoomResolutionCm * isq::length[cm]};
}

Position3D voxelCenter(long ix, long iy, long iz) {
    return Position3D{
        (static_cast<double>(ix) + 0.5) * kRoomResolutionCm * x_extent[cm],
        (static_cast<double>(iy) + 0.5) * kRoomResolutionCm * y_extent[cm],
        (static_cast<double>(iz) + 0.5) * kRoomResolutionCm * z_extent[cm]};
}

DroneConfigData droneConfigWithRadius(double radius_cm) {
    DroneConfigData config;
    config.radius = radius_cm * isq::length[cm];
    config.max_rotate = 90.0 * horizontal_angle[deg];
    config.max_advance = 10.0 * isq::length[cm];
    config.max_elevate = 10.0 * isq::length[cm];
    return config;
}

LidarConfigData roomLidarConfig() {
    LidarConfigData config;
    config.z_min = 5.0 * isq::length[cm];
    config.z_max = 50.0 * isq::length[cm];
    config.d = 2.5 * isq::length[cm];
    config.fov_circles = 2;
    return config;
}

MissionConfigData roomMissionConfig() {
    MissionConfigData config;
    config.max_steps = 6000;
    config.gps_resolution = kRoomResolutionCm * isq::length[cm];
    config.output_mapping_resolution_factor = 1.0;
    config.mission_bounds = roomMapConfig().boundaries;
    return config;
}

// Runs a full mission against a two-room synthetic map with the given opening
// size and drone radius, returning the resulting output_map so the caller can
// inspect specific voxels (e.g. "did it ever reach Room B?"). Drives the real
// MappingAlgorithmImpl through MissionControlImpl, which builds its own
// DroneControlImpl internally from the dependencies below (§7.2) -- Project 2
// built DroneControlImpl by hand and passed it in; Project 3 has no such
// external injection seam.
std::unique_ptr<Map3DImpl> runTwoRoomMission(long opening_w, long opening_h, double drone_radius_cm,
                                             MissionRunResult& out_result) {
    auto hidden_array = buildTwoRoomMap(kRoomNx, kRoomNy, kRoomNz, opening_w, opening_h);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, roomMapConfig());
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), roomMapConfig());

    const DroneConfigData drone = droneConfigWithRadius(drone_radius_cm);
    const LidarConfigData lidar = roomLidarConfig();
    const MissionConfigData mission = roomMissionConfig();

    // Start in the middle of Room A (x in [1, wall_x - 1]).
    const Position3D start_position = voxelCenter(kRoomNx / 4, kRoomNy / 2, kRoomNz / 2);
    MockGPS gps(start_position, Orientation{}, mission.gps_resolution);
    MockMovement movement(gps, *hidden_map, drone.radius);
    MockLidar lidar_sensor(lidar, *hidden_map, gps);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    const std::filesystem::path output_file =
        scratchDir() / ("two_room_" + std::to_string(opening_w) + "x" + std::to_string(opening_h) + "_" +
                       std::to_string(static_cast<int>(drone_radius_cm)) + "cm.npy");
    MissionControlImpl mission_control(MissionControlDependencies{
        mission, drone, lidar_sensor, gps, movement, *output_map, algorithm, output_file});

    out_result = mission_control.runMission();
    return output_map;
}

// True if any voxel in Room B (x > wall_x) is no longer Unmapped — i.e. the
// drone actually got inside Room B — checks only the *far* interior (beyond
// lidar z_max range from the doorway), which a beam fired through the
// opening from Room A cannot reach. Only physical entry can resolve this.
bool roomBHasBeenObserved(const Map3DImpl& output_map) {
    const long wall_x = kRoomNx / 2;
    const long far_x_start = wall_x + 7; // safely beyond z_max (5 voxels) from the doorway
    for (long ix = far_x_start; ix < kRoomNx - 1; ++ix) {
        for (long iy = 1; iy < kRoomNy - 1; ++iy) {
            for (long iz = 1; iz < kRoomNz - 1; ++iz) {
                if (output_map.atVoxel(voxelCenter(ix, iy, iz)) != VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

void expectFittingDroneReachesRoomB(long opening_w, long opening_h, double fits_radius_cm) {
    MissionRunResult result;
    const std::unique_ptr<Map3DImpl> output_map =
        runTwoRoomMission(opening_w, opening_h, fits_radius_cm, result);

    EXPECT_NE(result.status, MissionRunStatus::Error)
        << "Mission errored for a " << opening_w << "x" << opening_h << " opening with a "
        << fits_radius_cm << "cm-radius drone that should fit.";
    EXPECT_TRUE(roomBHasBeenObserved(*output_map))
        << "A " << fits_radius_cm << "cm-radius drone should fit through a " << opening_w << "x"
        << opening_h << " opening and observe at least part of Room B.";
}

void expectOversizedDroneCannotReachRoomB(long opening_w, long opening_h, double too_big_radius_cm) {
    MissionRunResult result;
    const std::unique_ptr<Map3DImpl> output_map =
        runTwoRoomMission(opening_w, opening_h, too_big_radius_cm, result);

    EXPECT_NE(result.status, MissionRunStatus::Error)
        << "Mission errored for a " << opening_w << "x" << opening_h << " opening with an oversized "
        << too_big_radius_cm << "cm-radius drone (it should safely stay in Room A, not crash).";
    EXPECT_FALSE(roomBHasBeenObserved(*output_map))
        << "A " << too_big_radius_cm << "cm-radius drone should NOT fit through a " << opening_w
        << "x" << opening_h << " opening, so Room B must remain entirely unobserved.";
}

// Real MappingAlgorithmFactory/MissionControlFactory wrapping the actual
// production implementations, for the one test below that exercises
// SimulationRunFactoryImpl end-to-end. Project 3 always builds
// Algorithm/MissionControl via injected factories (dlopen'd plugins in the
// real binary, §3) -- these local factories reproduce that shape for a
// direct, in-process test without going through the dlopen/registration path
// (RegistrationStub.cpp, see CMakeLists.txt, satisfies the REGISTER_* macros
// these two .cpp files still run at static-init time).
common::MappingAlgorithmFactory realMappingAlgorithmFactory() {
    return [](common::MappingAlgorithmDependencies dependencies) -> std::unique_ptr<IMappingAlgorithm> {
        return std::make_unique<MappingAlgorithmImpl>(std::move(dependencies));
    };
}

common::MissionControlFactory realMissionControlFactory() {
    return [](common::MissionControlDependencies dependencies) -> std::unique_ptr<IMissionControl> {
        return std::make_unique<MissionControlImpl>(std::move(dependencies));
    };
}

} // namespace

// ── 4x4 — the house's main entrance ──────────────────────────────────────────

// Radii below are chosen against the true continuous safety sphere -- a
// sphere-vs-voxel-volume (closest-point-on-AABB) test against the wall's real
// surface, not voxel-CENTER distance. Movement is still grid-aligned (the
// drone can only occupy one resolution_cm=10cm-wide cell at a time), so for an
// opening of width W voxels the *best available* cell is the one closest to
// the opening's true center, whose distance to the nearest wall's near
// surface is: W/2 * resolution_cm for odd W (an exactly-centered cell exists);
// (W-1)/2 * resolution_cm for even W (no exactly-centered cell -- the best
// available one is off-center by half a voxel). That distance is this
// scenario's maximum radius that can still pass through *at all* (and a
// sphere merely touching, at exactly that distance, still counts as a
// collision -- see UserCommon::sphereIntersectsAxisAlignedBox); each "fits"
// test below picks a radius comfortably under that maximum, and each
// "oversized" test picks one comfortably over it.
//
// An earlier version of these tests picked radii against voxel-CENTER
// distance instead (roughly ceil((W + 1) / 2) * resolution_cm from the
// opening's center) -- a materially looser bound that was only ever correct
// for the algorithm's old, buggy safety check. Two "fits" cases (4x4/15cm,
// 2x2/8cm) picked radii between the true continuous-surface maximum and that
// looser voxel-center one; correcting the safety geometry to the true
// continuous sphere (this task's whole point) makes those two specific radii
// genuinely too large to fit through their openings, so both are lowered here
// to genuinely-fitting values. No opening size, "oversized" radius, or any
// other scenario in this file changed.

TEST(Internal, FourByFourEntranceFittingDroneReachesSecondRoom) {
    // True maximum: (4-1)/2 * 10cm = 15cm (a sphere touching it is itself
    // rejected). 13cm leaves a comfortable margin.
    expectFittingDroneReachesRoomB(4, 4, 13.0);
}

TEST(Internal, FourByFourEntranceOversizedDroneCannotReachSecondRoom) {
    expectOversizedDroneCannotReachRoomB(4, 4, 35.0);
}

// ── 3x3 — one of the second-floor rooms ──────────────────────────────────────

TEST(Internal, ThreeByThreeEntranceFittingDroneReachesSecondRoom) {
    // True maximum: 3/2 * 10cm = 15cm (odd width -- an exactly-centered cell
    // exists). 12cm already leaves a comfortable margin under the true
    // continuous-surface maximum, not just the old voxel-center one, so this
    // radius did not need to change.
    expectFittingDroneReachesRoomB(3, 3, 12.0);
}

TEST(Internal, ThreeByThreeEntranceOversizedDroneCannotReachSecondRoom) {
    expectOversizedDroneCannotReachRoomB(3, 3, 30.0);
}

// ── 2x2 — the other second-floor room ────────────────────────────────────────

TEST(Internal, TwoByTwoEntranceFittingDroneReachesSecondRoom) {
    // True maximum: (2-1)/2 * 10cm = 5cm (even width -- no exactly-centered
    // cell; the best available one is off-center by half a voxel). 4cm
    // leaves a 1cm margin -- the same radius already used and verified below
    // for the similarly 2-voxel-wide TwoByOne opening.
    expectFittingDroneReachesRoomB(2, 2, 4.0);
}

TEST(Internal, TwoByTwoEntranceOversizedDroneCannotReachSecondRoom) {
    expectOversizedDroneCannotReachRoomB(2, 2, 25.0);
}

// ── 2x1 — the secret roof entrance ───────────────────────────────────────────

TEST(Internal, TwoByOneSecretEntranceFittingDroneReachesSecondRoom) {
    expectFittingDroneReachesRoomB(2, 1, 4.0);
}

TEST(Internal, TwoByOneSecretEntranceOversizedDroneCannotReachSecondRoom) {
    expectOversizedDroneCannotReachRoomB(2, 1, 20.0);
}

// ── 1x1 — the nested room's roof opening ─────────────────────────────────────

TEST(Internal, OneByOneNestedRoomEntranceFittingDroneReachesSecondRoom) {
    expectFittingDroneReachesRoomB(1, 1, 4.0);
}

TEST(Internal, OneByOneNestedRoomEntranceOversizedDroneCannotReachSecondRoom) {
    expectOversizedDroneCannotReachRoomB(1, 1, 20.0);
}

// ── Baseline flows from the original Phase 5 plan ────────────────────────────

TEST(Internal, RealAlgorithmFlowOnSmallKnownMapDoesNotCrash) {
    SimulationConfigData simulation;
    simulation.map_filename = "data_maps/single_voxel_x4_y4_z4.npy";
    simulation.map_resolution = 10.0 * isq::length[cm];
    simulation.initial_drone_position =
        Position3D{5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]};
    simulation.initial_angle = 0.0 * horizontal_angle[deg];

    MissionConfigData mission;
    mission.max_steps = 500;
    mission.gps_resolution = 10.0 * isq::length[cm];
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = MappingBounds{
        0.0 * x_extent[cm], 50.0 * x_extent[cm],
        0.0 * y_extent[cm], 50.0 * y_extent[cm],
        0.0 * z_extent[cm], 50.0 * z_extent[cm]};

    DroneConfigData drone;
    drone.radius = 4.0 * isq::length[cm];
    drone.max_rotate = 45.0 * horizontal_angle[deg];
    drone.max_advance = 10.0 * isq::length[cm];
    drone.max_elevate = 10.0 * isq::length[cm];

    LidarConfigData lidar;
    lidar.z_min = 5.0 * isq::length[cm];
    lidar.z_max = 50.0 * isq::length[cm];
    lidar.d = 2.5 * isq::length[cm];
    lidar.fov_circles = 2;

    SimulationRunFactoryImpl factory(realMappingAlgorithmFactory(), realMissionControlFactory(), /*verbose=*/false);
    const std::unique_ptr<ISimulationRun> run =
        factory.create(simulation, mission, drone, lidar, scratchDir());
    const SimulationResult result = run->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_GE(result.mission_score, 0.0);
    EXPECT_TRUE(std::filesystem::exists(result.output_map_file))
        << "the real algorithm flow must save its output map to the file it reports; missing: "
        << result.output_map_file;
}

TEST(Internal, MockAlgorithmAlwaysHoveringHitsMaxSteps) {
    auto hidden_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), roomMapConfig());
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), roomMapConfig());

    const DroneConfigData drone = droneConfigWithRadius(5.0);
    const LidarConfigData lidar = roomLidarConfig();
    MissionConfigData mission = roomMissionConfig();
    mission.max_steps = 50;

    MockGPS gps(voxelCenter(2, 2, 2), Orientation{}, mission.gps_resolution);
    MockMovement movement(gps, *hidden_map, drone.radius);
    MockLidar lidar_sensor(lidar, *hidden_map, gps);

    ::testing::NiceMock<test::GMockIMappingAlgorithm> algorithm(
        MappingAlgorithmDependencies{mission, lidar, drone, *output_map});
    ON_CALL(algorithm, nextStep(::testing::_, ::testing::_))
        .WillByDefault(::testing::Return(MappingStepCommand{}));

    const std::filesystem::path output_file = scratchDir() / "mock_algorithm_hover.npy";
    MissionControlImpl mission_control(MissionControlDependencies{
        mission, drone, lidar_sensor, gps, movement, *output_map, algorithm, output_file});

    const MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, mission.max_steps);
}

// Regression for a DroneControlImpl/MockMovement direction-math inconsistency (Optional
// Common-Issues row 12): DroneControlImpl used to predict an Advance's expected post-movement
// position by zeroing a negligible cos/sin residue at an axis-aligned heading (e.g.
// cos(90deg) ~ 6.12e-17), while the real MockMovement::advance() applied that same heading's raw,
// un-zeroed cos/sin -- a real, if tiny, position disagreement between prediction and actual
// movement that had nothing to do with GPS being wrong. At a large enough distance that
// disagreement (~6e-8cm here, at distance=1e9cm) exceeds row 12's strict machine-epsilon
// tolerance, so with the old, inconsistent math this perfectly legitimate Advance would have been
// wrongly reported as Error. Both sides now share
// user_common_322889890_315113738::advanceDirection(), so they can never disagree -- this uses
// the real MockMovement + MockGPS (not a mocked IDroneMovement) so that agreement is actually
// exercised, not merely assumed.
//
// Talks to DroneControlImpl directly (not through MissionControlImpl) so a mismatch surfaces as
// this step's own DroneStepStatus::Error instead of being swallowed by MissionControl's row-9
// log-and-continue handling. The hidden/output map bounds are huge purely to give the drone room
// to travel this far; its resolution is scaled to match so MockMovement's collision-path sampling
// stays cheap (tens of samples, not tens of millions), and every voxel stays Unmapped (never
// Occupied), so no collision is ever possible along the path.
TEST(Internal, RealMockMovementAdvanceAtOrthogonalHeadingAgreesWithDroneControlExpectedPosition) {
    const MappingBounds huge_bounds{
        0.0 * x_extent[cm], 2.0e9 * x_extent[cm], 0.0 * y_extent[cm], 2.0e9 * y_extent[cm],
        0.0 * z_extent[cm], 2.0e9 * z_extent[cm]};
    const MapConfig huge_map_config{huge_bounds, Position3D{}, 2.0e8 * isq::length[cm]};

    auto hidden_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), huge_map_config);
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), huge_map_config);

    DroneConfigData drone;
    drone.radius = 5.0 * isq::length[cm];
    drone.max_rotate = 90.0 * horizontal_angle[deg];
    drone.max_advance = 1.0e10 * isq::length[cm]; // large enough this Advance is never split
    drone.max_elevate = 1.0e10 * isq::length[cm];

    LidarConfigData lidar;
    lidar.z_min = 5.0 * isq::length[cm];
    lidar.z_max = 50.0 * isq::length[cm];
    lidar.d = 2.5 * isq::length[cm];
    lidar.fov_circles = 2;

    MissionConfigData mission;
    mission.max_steps = 10;
    mission.gps_resolution = 10.0 * isq::length[cm];
    mission.output_mapping_resolution_factor = 1.0;

    const Position3D start_position{
        1.0e5 * x_extent[cm], 1.0e5 * y_extent[cm], 1.0e5 * z_extent[cm]};
    const Orientation orthogonal_heading{90.0 * horizontal_angle[deg], AltitudeAngle{}};
    MockGPS gps(start_position, orthogonal_heading, mission.gps_resolution);
    MockMovement movement(gps, *hidden_map, drone.radius);

    ::testing::NiceMock<test::GMockILidar> lidar_mock;
    EXPECT_CALL(lidar_mock, scan(::testing::_)).Times(0);

    MappingStepCommand advance_command;
    advance_command.movement = MovementCommand{};
    advance_command.movement->type = MovementCommandType::Advance;
    advance_command.movement->distance = 1.0e9 * isq::length[cm];
    // No scan_orientation: this test is purely about position agreement, not scanning.

    ::testing::NiceMock<test::GMockIMappingAlgorithm> algorithm(
        MappingAlgorithmDependencies{mission, lidar, drone, *output_map});
    EXPECT_CALL(algorithm, nextStep(::testing::_, ::testing::_))
        .WillOnce(::testing::Return(advance_command));

    DroneControlImpl control(drone, mission, lidar_mock, gps, movement, *output_map, algorithm);

    const DroneStepResult result = control.step();

    EXPECT_EQ(result.status, DroneStepStatus::Continue) << "message: " << result.message;
}
