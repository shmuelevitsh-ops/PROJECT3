// Migrated from Project 2 (FILES PROJECT 2/tests/components/mapping_algorithm_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3): namespaces/
// includes, plus one mechanical constructor-shape change forced by the new architecture --
// MappingAlgorithmImpl's constructor takes a single MappingAlgorithmDependencies aggregate
// instead of four positional arguments (§7.2); every call site below wraps its same four
// arguments in that aggregate, nothing else changed. Uses the real Simulator-owned Map3DImpl
// as a concrete IMap3D (test-only dependency across the module boundary, decided in Phase 3
// specifically so this suite could keep exercising real map geometry -- see
// PROJ2_TESTS_PLAN.md §6 and PROJ2_TESTS_EXECUTION_PLAN.md Phase 3 task 1); introduces no
// production dependency from Algorithm on Simulator.

#include <Algorithm/MappingAlgorithmImpl.h>
#include <Simulator/Map3DImpl.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

using namespace common;
using namespace common::types;
using namespace algorithm_322889890_315113738;
using simulator::Map3DImpl;

namespace {

Position3D pos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

MappingBounds boundsFromVoxelCount(long count, double resolution_cm) {
    const double extent_cm = static_cast<double>(count) * resolution_cm;
    return MappingBounds{
        0.0 * x_extent[cm], extent_cm * x_extent[cm],
        0.0 * y_extent[cm], extent_cm * y_extent[cm],
        0.0 * z_extent[cm], extent_cm * z_extent[cm]};
}

// Small grid (configurable voxel count per axis) at 10cm resolution. A 1cm drone radius keeps
// the safety-sphere check effectively single-voxel, so tests can reason about individual cells.
MapConfig gridConfig(long voxels_per_axis) {
    return MapConfig{boundsFromVoxelCount(voxels_per_axis, 10.0), Position3D{}, 10.0 * isq::length[cm]};
}

DroneConfigData droneConfig() {
    DroneConfigData config;
    config.radius = 1.0 * isq::length[cm];
    config.max_rotate = 30.0 * horizontal_angle[deg];
    config.max_advance = 5.0 * isq::length[cm];
    config.max_elevate = 5.0 * isq::length[cm];
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

MissionConfigData missionConfig() {
    MissionConfigData config;
    config.max_steps = 1000;
    config.gps_resolution = 1.0 * isq::length[cm];
    config.output_mapping_resolution_factor = 1.0;
    return config;
}

std::unique_ptr<Map3DImpl> freshMap(const MapConfig& config) {
    return std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), config);
}

// Voxel center for index (ix, iy, iz) on a 10cm-resolution, zero-offset grid (matches gridConfig()).
Position3D voxelCenter(long ix, long iy, long iz) {
    return pos((static_cast<double>(ix) + 0.5) * 10.0, (static_cast<double>(iy) + 0.5) * 10.0,
               (static_cast<double>(iz) + 0.5) * 10.0);
}

// Mimics what MockMovement / DroneControlImpl would do with a returned MovementCommand, so
// component tests can drive MappingAlgorithmImpl across several steps without a real
// IDroneMovement. Hover and unset commands leave the state unchanged.
DroneState applyMovement(DroneState state, const std::optional<MovementCommand>& movement) {
    if (!movement) {
        return state;
    }
    switch (movement->type) {
        case MovementCommandType::Rotate: {
            const HorizontalAngle signed_angle =
                movement->rotation == RotationDirection::Left ? movement->angle : -movement->angle;
            state.heading.horizontal = state.heading.horizontal + signed_angle;
            break;
        }
        case MovementCommandType::Advance: {
            const double heading_rad =
                state.heading.horizontal.force_numerical_value_in(deg) * M_PI / 180.0;
            const double distance_cm = movement->distance.force_numerical_value_in(cm);
            state.position.x = state.position.x + distance_cm * std::cos(heading_rad) * x_extent[cm];
            state.position.y = state.position.y + distance_cm * std::sin(heading_rad) * y_extent[cm];
            break;
        }
        case MovementCommandType::Elevate: {
            state.position.z = state.position.z + movement->distance.force_numerical_value_in(cm) * z_extent[cm];
            break;
        }
        case MovementCommandType::Hover:
            break;
    }
    return state;
}

// Drives `algorithm` until it reports a non-Working status or `max_iterations` is exceeded.
// The map is never updated from `latest_scan` (no real ScanResultToVoxels in these component
// tests) - callers that need scans to resolve must update the injected map directly.
AlgorithmStatus runUntilDone(MappingAlgorithmImpl& algorithm, DroneState state, int max_iterations) {
    AlgorithmStatus status = AlgorithmStatus::Working;
    for (int i = 0; i < max_iterations && status == AlgorithmStatus::Working; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        status = command.status;
        state = applyMovement(state, command.movement);
        state.step_index += 1;
    }
    return status;
}

void fillAllEmpty(Map3DImpl& map, long voxels_per_axis) {
    for (long ix = 0; ix < voxels_per_axis; ++ix) {
        for (long iy = 0; iy < voxels_per_axis; ++iy) {
            for (long iz = 0; iz < voxels_per_axis; ++iz) {
                map.set(voxelCenter(ix, iy, iz), VoxelOccupancy::Empty);
            }
        }
    }
}

// --- Generalized helpers below: used by tests that vary offset, resolution, drone radius, lidar
// range, or movement limits away from the simple 10cm/zero-offset/tiny-radius defaults above. ---

// A grid of `voxels_per_axis` cells per axis at an arbitrary resolution and offset.
MapConfig customGridConfig(long voxels_per_axis, double resolution_cm, double offset_x_cm = 0.0,
                            double offset_y_cm = 0.0, double offset_z_cm = 0.0) {
    const double extent_cm = static_cast<double>(voxels_per_axis) * resolution_cm;
    return MapConfig{
        MappingBounds{offset_x_cm * x_extent[cm], (offset_x_cm + extent_cm) * x_extent[cm],
                      offset_y_cm * y_extent[cm], (offset_y_cm + extent_cm) * y_extent[cm],
                      offset_z_cm * z_extent[cm], (offset_z_cm + extent_cm) * z_extent[cm]},
        Position3D{offset_x_cm * x_extent[cm], offset_y_cm * y_extent[cm], offset_z_cm * z_extent[cm]},
        resolution_cm * isq::length[cm]};
}

// Voxel center for index (ix, iy, iz) under an arbitrary MapConfig (accounts for its offset and
// resolution, unlike the fixed-10cm-zero-offset `voxelCenter()` above).
Position3D voxelCenterIn(const MapConfig& config, long ix, long iy, long iz) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    return Position3D{
        config.offset.x + (static_cast<double>(ix) + 0.5) * resolution_cm * x_extent[cm],
        config.offset.y + (static_cast<double>(iy) + 0.5) * resolution_cm * y_extent[cm],
        config.offset.z + (static_cast<double>(iz) + 0.5) * resolution_cm * z_extent[cm]};
}

void fillAllEmptyIn(Map3DImpl& map, const MapConfig& config, long voxels_per_axis) {
    for (long ix = 0; ix < voxels_per_axis; ++ix) {
        for (long iy = 0; iy < voxels_per_axis; ++iy) {
            for (long iz = 0; iz < voxels_per_axis; ++iz) {
                map.set(voxelCenterIn(config, ix, iy, iz), VoxelOccupancy::Empty);
            }
        }
    }
}

DroneConfigData customDroneConfig(double radius_cm, double max_rotate_deg, double max_advance_cm,
                                   double max_elevate_cm) {
    DroneConfigData config;
    config.radius = radius_cm * isq::length[cm];
    config.max_rotate = max_rotate_deg * horizontal_angle[deg];
    config.max_advance = max_advance_cm * isq::length[cm];
    config.max_elevate = max_elevate_cm * isq::length[cm];
    return config;
}

LidarConfigData customLidarConfig(double z_min_cm, double z_max_cm) {
    LidarConfigData config;
    config.z_min = z_min_cm * isq::length[cm];
    config.z_max = z_max_cm * isq::length[cm];
    config.d = 1.0 * isq::length[cm];
    config.fov_circles = 1;
    return config;
}

// Discrete grid index, mirroring (but independent of) the production VoxelIndex - kept local to
// this test file so tests can reason about positions/distances without depending on
// MappingAlgorithmImpl.cpp's private implementation details.
struct TestVoxelIndex {
    long ix = 0;
    long iy = 0;
    long iz = 0;
};

TestVoxelIndex toIndexIn(const MapConfig& config, const Position3D& pos) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    return TestVoxelIndex{
        static_cast<long>(std::floor((pos.x - config.offset.x).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.y - config.offset.y).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.z - config.offset.z).force_numerical_value_in(cm) / resolution_cm)),
    };
}

// Independent test-side sphere-vs-voxel-volume overlap check: computes the closest point of
// voxel `idx`'s own axis-aligned volume to `pos` and reports whether that point lies within
// `radius_cm` of `pos` -- true sphere-vs-voxel-VOLUME overlap, not voxel-center-to-`pos`
// distance. Deliberately a separate, from-scratch calculation rather than a call into
// production's UserCommon::sphereIntersectsAxisAlignedBox, so this oracle doesn't end up
// validating production geometry against the exact same implementation it is testing.
bool voxelVolumeOverlapsSphere(const MapConfig& config, const TestVoxelIndex& idx, const Position3D& pos,
                                double radius_cm) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double min_x = config.offset.x.force_numerical_value_in(cm) + static_cast<double>(idx.ix) * resolution_cm;
    const double min_y = config.offset.y.force_numerical_value_in(cm) + static_cast<double>(idx.iy) * resolution_cm;
    const double min_z = config.offset.z.force_numerical_value_in(cm) + static_cast<double>(idx.iz) * resolution_cm;
    const double max_x = min_x + resolution_cm;
    const double max_y = min_y + resolution_cm;
    const double max_z = min_z + resolution_cm;

    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);

    const double closest_x = std::clamp(px, min_x, max_x);
    const double closest_y = std::clamp(py, min_y, max_y);
    const double closest_z = std::clamp(pz, min_z, max_z);

    const double dx = px - closest_x;
    const double dy = py - closest_y;
    const double dz = pz - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= radius_cm * radius_cm;
}

// Asserts the spec's safety rule directly against the map's public API (atVoxel/isInBounds):
// every voxel whose axis-aligned VOLUME overlaps a sphere of `radius_cm` centered at `pos` --
// not merely whose center lies within `radius_cm` of `pos` -- must be Empty, and `pos` itself
// must be in bounds. Used to verify that movements proposed by the algorithm never end up
// somewhere unsafe - not by inspecting algorithm internals, only by querying the injected map.
void expectPositionSafe(const IMap3D& map, const MapConfig& config, double radius_cm, const Position3D& pos) {
    ASSERT_TRUE(map.isInBounds(pos)) << "drone position is outside the map's configured boundaries";
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    // +1 pads the plain radius-derived voxel range: `pos` need not sit at any voxel's own
    // center, so a voxel whose center is just past that range can still have its volume within
    // reach of the true, continuous sphere.
    const long voxel_radius = static_cast<long>(std::ceil(radius_cm / resolution_cm)) + 1;
    const TestVoxelIndex idx = toIndexIn(config, pos);

    for (long dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (long dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (long dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                const TestVoxelIndex neighbor{idx.ix + dx, idx.iy + dy, idx.iz + dz};
                if (!voxelVolumeOverlapsSphere(config, neighbor, pos, radius_cm)) {
                    continue;
                }
                const Position3D neighbor_center = voxelCenterIn(config, neighbor.ix, neighbor.iy, neighbor.iz);
                EXPECT_EQ(map.atVoxel(neighbor_center), VoxelOccupancy::Empty)
                    << "voxel at offset (" << dx << "," << dy << "," << dz << ") from the drone's "
                       "position has a volume overlapping the drone's safety sphere but is not Empty";
            }
        }
    }
}

// Checks every intermediate position along the straight-line movement from `from` to `to`, at
// the same 0.1*resolution sampling granularity MockMovement::pathCollides() uses against the
// *hidden* map, but here run test-side against the algorithm's own (known) `map` -- independent
// of MockMovement entirely, since these component tests never construct one. This matters once
// movement merging (buildMovementQueue) is in play: a single Advance/Elevate command can now span
// several voxels, so checking only its endpoint (as a single expectPositionSafe() call would) can
// no longer independently confirm every voxel *along the way* was actually safe -- a bug that
// merged across an unvalidated or unsafe intermediate voxel would move the endpoint check's own
// sphere off of it without ever sampling it.
void expectPathSafe(const IMap3D& map, const MapConfig& config, double radius_cm, const Position3D& from,
                     const Position3D& to) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double dx = (to.x - from.x).force_numerical_value_in(cm);
    const double dy = (to.y - from.y).force_numerical_value_in(cm);
    const double dz = (to.z - from.z).force_numerical_value_in(cm);
    const double total_distance_cm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (resolution_cm > 0.0 && total_distance_cm > 0.0) {
        const double step_cm = 0.1 * resolution_cm;
        const long num_steps = static_cast<long>(std::floor(total_distance_cm / step_cm));
        for (long i = 0; i <= num_steps; ++i) {
            const double t = (static_cast<double>(i) * step_cm) / total_distance_cm;
            const Position3D sample{
                from.x + t * dx * x_extent[cm],
                from.y + t * dy * y_extent[cm],
                from.z + t * dz * z_extent[cm],
            };
            SCOPED_TRACE("intermediate sample at t=" + std::to_string(t));
            expectPositionSafe(map, config, radius_cm, sample);
        }
    }
    // Always check the exact destination too, even when it is not an exact multiple of the
    // sampling step.
    expectPositionSafe(map, config, radius_cm, to);
}

// Drives `algorithm` like runUntilDone(), but additionally asserts - after every step that
// actually changes position - that the *entire* movement (not just its destination -- see
// expectPathSafe() above) satisfies the sphere-safety rule against the *current* map contents.
// This verifies the full movement path is safe, not only final destinations: a bug that skips
// (or, post-merging, spans) an unsafe intermediate voxel would be caught here on the very step
// that proposes it.
AlgorithmStatus runUntilDoneVerifyingPathSafety(MappingAlgorithmImpl& algorithm, DroneState state,
                                                 const IMap3D& map, double radius_cm, int max_iterations) {
    const MapConfig config = map.getMapConfig();
    AlgorithmStatus status = AlgorithmStatus::Working;
    for (int i = 0; i < max_iterations && status == AlgorithmStatus::Working; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        status = command.status;
        const DroneState next_state = applyMovement(state, command.movement);
        const bool position_changed =
            command.movement.has_value() &&
            (command.movement->type == MovementCommandType::Advance ||
             command.movement->type == MovementCommandType::Elevate);
        if (position_changed) {
            SCOPED_TRACE("nextStep() call #" + std::to_string(i));
            expectPathSafe(map, config, radius_cm, state.position, next_state.position);
        }
        state = next_state;
        state.step_index += 1;
    }
    return status;
}

} // namespace

TEST(MappingAlgorithm, FirstCallOnFreshMapDoesNotThrowAndProducesAnAction) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    MappingStepCommand command;
    ASSERT_NO_THROW(command = algorithm.nextStep(state, nullptr));

    EXPECT_EQ(command.status, AlgorithmStatus::Working);
    EXPECT_TRUE(command.movement.has_value() || command.scan_orientation.has_value())
        << "the very first call must do something rather than hover indefinitely";
}

TEST(MappingAlgorithm, DoesNotAdvanceIntoOccupiedForwardVoxel) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::Occupied);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "the only forward voxel was Occupied; the algorithm must not have planned a move into it";
}

TEST(MappingAlgorithm, DoesNotAdvanceIntoUnmappedForwardVoxel) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config); // forward voxel is Unmapped by default.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value());
    EXPECT_TRUE(command.scan_orientation.has_value())
        << "an Unmapped forward voxel must be scanned, never moved into";
}

TEST(MappingAlgorithm, DoesNotAdvanceIntoPotentiallyOccupiedForwardVoxel) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::PotentiallyOccupied);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value());
    EXPECT_TRUE(command.scan_orientation.has_value());
}

// --- Sweep retry budget (kMaxSweepScanAttempts) -------------------------------------------------

TEST(MappingAlgorithm, BlockedSweepCandidateIsScannedDirectlyBeforeFallingThroughToFrontier) {
    // On a fresh (all-Unmapped) grid, the very first sweep candidate is the lawnmower's next cell
    // (1,0,0) (the +x neighbor of the start cell). A Sweep-phase retry budget of at least one
    // attempt means that candidate must be scanned *directly* by Sweep on this very first call --
    // not skipped straight to a Frontier-phase detour, whose independent BFS (iterating dz
    // innermost) would instead pick a *different* nearby Unmapped cell, (0,0,1), as its target.
    // A bug that sets the retry budget to zero would skip the direct Sweep scan entirely, so the
    // first command would aim at (0,0,1) (straight up, altitude ~90deg) instead of (1,0,0) (level,
    // altitude ~0deg) -- a clean, observable substitution of *which* cell gets scanned first.
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config); // every cell Unmapped by default.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_FALSE(command.movement.has_value())
        << "the immediate sweep candidate is Unmapped; it must be scanned, not moved into";
    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_NEAR(command.scan_orientation->altitude.force_numerical_value_in(deg), 0.0, 1.0)
        << "the Sweep phase must scan its own immediate +x candidate (1,0,0) directly, not fall "
           "through to a Frontier-phase target such as (0,0,1)";
}

TEST(MappingAlgorithm, ChunkedAdvanceSumsToOneResolutionStep) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::Empty); // safe to advance into.
    DroneConfigData drone = droneConfig();
    drone.max_advance = 5.0 * isq::length[cm]; // half of the 10cm resolution step.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};

    const MappingStepCommand first = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(first.movement.has_value());
    EXPECT_EQ(first.movement->type, MovementCommandType::Advance);
    EXPECT_LE(first.movement->distance.force_numerical_value_in(cm), 5.0 + 1e-6);

    const MappingStepCommand second = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(second.movement.has_value());
    EXPECT_EQ(second.movement->type, MovementCommandType::Advance);
    EXPECT_LE(second.movement->distance.force_numerical_value_in(cm), 5.0 + 1e-6);

    const double total_cm =
        first.movement->distance.force_numerical_value_in(cm) + second.movement->distance.force_numerical_value_in(cm);
    EXPECT_NEAR(total_cm, 10.0, 1e-6);
}

TEST(MappingAlgorithm, AdvanceWithMaxAdvanceAtLeastResolutionMovesInASingleStep) {
    // The chunking-tested-above case (max_advance < resolution) needs >1 chunk; this is the
    // opposite boundary (max_advance >= resolution), where a single Advance must already cover
    // the whole resolution step. A chunking-formula bug that assumes a remainder is always left
    // over (e.g. always chunking by max_advance regardless of how much distance remains) would
    // either request more than one resolution step here or split this into a needless second
    // call.
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::Empty); // safe to advance into.
    DroneConfigData drone = droneConfig();
    drone.max_advance = 10.0 * isq::length[cm]; // == the 10cm resolution step, not less than it.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 10.0, 1e-6)
        << "a single Advance should cover the entire 10cm resolution step when max_advance already "
           "allows it, not an under- or over-shoot from an incorrect chunk-size calculation";
}

TEST(MappingAlgorithm, ChunkedRotateSumsToRequiredTurn) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::Empty); // safe to advance into, due east.
    DroneConfigData drone = droneConfig();
    drone.max_rotate = 30.0 * horizontal_angle[deg]; // the required 90deg turn needs 3 chunks.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    // The sweep's first move is always due east (see initSweepBounds), but the drone starts
    // facing north - it must rotate 90 degrees before it can advance toward that cell.
    DroneState state{voxelCenter(0, 0, 0), Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 0};
    double total_signed_deg = 0.0;
    bool saw_advance = false;
    for (int i = 0; i < 10 && !saw_advance; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value()) << "expected a Rotate/Advance sequence, got neither";
        if (command.movement->type == MovementCommandType::Rotate) {
            const double signed_deg = command.movement->rotation == RotationDirection::Left
                                           ? command.movement->angle.force_numerical_value_in(deg)
                                           : -command.movement->angle.force_numerical_value_in(deg);
            EXPECT_LE(std::fabs(signed_deg), 30.0 + 1e-6);
            total_signed_deg += signed_deg;
        } else if (command.movement->type == MovementCommandType::Advance) {
            saw_advance = true;
        }
        state = applyMovement(state, command.movement);
    }

    EXPECT_TRUE(saw_advance) << "rotation chunks must eventually be followed by the advance";
    EXPECT_NEAR(std::fabs(total_signed_deg), 90.0, 1e-3);
}

TEST(MappingAlgorithm, ScanOrientationIsRelativeToHeadingNotAbsolute) {
    const auto map_config = gridConfig(5);

    const auto map_zero = freshMap(map_config);
    MappingAlgorithmImpl algorithm_zero(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map_zero});
    const DroneState state_zero{voxelCenter(0, 0, 0), Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 0};
    const MappingStepCommand command_zero = algorithm_zero.nextStep(state_zero, nullptr);
    ASSERT_TRUE(command_zero.scan_orientation.has_value());

    const auto map_rotated = freshMap(map_config);
    MappingAlgorithmImpl algorithm_rotated(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map_rotated});
    const DroneState state_rotated{voxelCenter(0, 0, 0),
                                    Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 0};
    const MappingStepCommand command_rotated = algorithm_rotated.nextStep(state_rotated, nullptr);
    ASSERT_TRUE(command_rotated.scan_orientation.has_value());

    // The target voxel is due east in both scenarios; the relative scan angle must shift by
    // the same amount as the heading difference (90 degrees), not stay fixed in world space.
    const double horizontal_zero_deg = command_zero.scan_orientation->horizontal.force_numerical_value_in(deg);
    const double horizontal_rotated_deg = command_rotated.scan_orientation->horizontal.force_numerical_value_in(deg);
    EXPECT_NEAR(horizontal_zero_deg, 0.0, 1.0);
    EXPECT_NEAR(horizontal_rotated_deg, -90.0, 1.0);
}

TEST(MappingAlgorithm, OutputMapIsAuthoritativeOverPreviousScanRequest) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});
    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};

    const MappingStepCommand first = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(first.scan_orientation.has_value()) << "forward voxel starts Unmapped, so a scan is expected";

    // Simulate what DroneControlImpl + ScanResultToVoxels would have done after that scan.
    map->set(voxelCenter(1, 0, 0), VoxelOccupancy::Occupied);

    const MappingStepCommand second = algorithm.nextStep(state, nullptr);
    EXPECT_FALSE(second.movement.has_value())
        << "the map now shows the forward voxel as Occupied; the algorithm must not move into it";
}

TEST(MappingAlgorithm, LatestScanArgumentIsOptionalAndDoesNotAffectCorrectness) {
    const auto map_config = gridConfig(5);
    const auto map = freshMap(map_config);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});
    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};

    LidarScanResult scan;
    scan.push_back(LidarHit{0.0 * cm, Orientation{}});
    scan.push_back(LidarHit{std::numeric_limits<double>::max() * cm, Orientation{}});

    MappingStepCommand command;
    ASSERT_NO_THROW(command = algorithm.nextStep(state, &scan));
    EXPECT_EQ(command.status, AlgorithmStatus::Working);
    EXPECT_TRUE(command.movement.has_value() || command.scan_orientation.has_value());
}

TEST(MappingAlgorithm, FinishedWhenEveryVoxelIsEmpty) {
    constexpr long kVoxelsPerAxis = 2;
    const auto map_config = gridConfig(kVoxelsPerAxis);
    const auto map = freshMap(map_config);
    fillAllEmpty(*map, kVoxelsPerAxis);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_EQ(status, AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, FinishedWithUnmappableVoxelsWhenATargetNeverResolves) {
    constexpr long kVoxelsPerAxis = 2;
    const auto map_config = gridConfig(kVoxelsPerAxis);
    const auto map = freshMap(map_config);
    fillAllEmpty(*map, kVoxelsPerAxis);
    // Leave exactly one voxel Unmapped; since this component test never applies a real scan
    // result to the map, it can never be resolved - the algorithm must eventually give up on it.
    map->set(voxelCenter(1, 1, 1), VoxelOccupancy::Unmapped);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_EQ(status, AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, PotentiallyOccupiedVoxelIsNotConsideredFullyMapped) {
    constexpr long kVoxelsPerAxis = 2;
    const auto map_config = gridConfig(kVoxelsPerAxis);
    const auto map = freshMap(map_config);
    fillAllEmpty(*map, kVoxelsPerAxis);
    map->set(voxelCenter(1, 1, 1), VoxelOccupancy::PotentiallyOccupied);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenter(0, 0, 0), Orientation{}, 0};
    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_NE(status, AlgorithmStatus::Finished)
        << "a reachable PotentiallyOccupied voxel must prevent Finished status";
}

// --- Sphere safety with a non-trivial drone radius ---------------------------------------------
//
// A 7-voxel-per-axis grid keeps the candidate well away from the boundary, so every voxel in its
// safety sphere is itself in-bounds and individually controllable. A 12cm radius (resolution
// 10cm) covers the candidate's 6 face neighbors (distance 10cm) but excludes edge/corner
// neighbors (>=14.14cm) - so exactly one neighbor can be made unsafe without disturbing the rest.

namespace {

// Builds a 7^3 grid where the start cell (3,3,3) and the candidate cell (4,3,3) are Empty, and
// every face-neighbor of the candidate is Empty too, except (4,4,3) which is set to `bad_value`.
// The candidate is itself Empty, so a safety rule that checked only the candidate cell (as a
// radius <= resolution rule would) would wrongly call it safe.
std::unique_ptr<Map3DImpl> sphereSafetyMap(VoxelOccupancy bad_value) {
    const auto config = customGridConfig(7, 10.0);
    auto map = freshMap(config);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    map->set(vc(3, 3, 3), VoxelOccupancy::Empty); // start
    map->set(vc(4, 3, 3), VoxelOccupancy::Empty); // candidate itself
    map->set(vc(5, 3, 3), VoxelOccupancy::Empty); // +x face neighbor
    map->set(vc(4, 2, 3), VoxelOccupancy::Empty); // -y face neighbor
    map->set(vc(4, 3, 4), VoxelOccupancy::Empty); // +z face neighbor
    map->set(vc(4, 3, 2), VoxelOccupancy::Empty); // -z face neighbor
    map->set(vc(4, 4, 3), bad_value);             // +y face neighbor: the one unsafe voxel
    return map;
}

} // namespace

TEST(MappingAlgorithm, SphereSafetyRejectsEmptyCandidateWithOccupiedNeighborInRadius) {
    const auto map = sphereSafetyMap(VoxelOccupancy::Occupied);
    const auto drone = customDroneConfig(/*radius_cm=*/12.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenterIn(map->getMapConfig(), 3, 3, 3), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "candidate cell is itself Empty, but an Occupied voxel inside the drone's safety "
           "sphere must still block the move";
}

TEST(MappingAlgorithm, SphereSafetyRejectsEmptyCandidateWithUnmappedNeighborInRadius) {
    const auto map = sphereSafetyMap(VoxelOccupancy::Unmapped);
    const auto drone = customDroneConfig(12.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenterIn(map->getMapConfig(), 3, 3, 3), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "candidate cell is itself Empty, but an Unmapped voxel inside the drone's safety "
           "sphere must still block the move";
}

TEST(MappingAlgorithm, SphereSafetyRejectsEmptyCandidateWithPotentiallyOccupiedNeighborInRadius) {
    const auto map = sphereSafetyMap(VoxelOccupancy::PotentiallyOccupied);
    const auto drone = customDroneConfig(12.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenterIn(map->getMapConfig(), 3, 3, 3), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "candidate cell is itself Empty, but a PotentiallyOccupied voxel inside the drone's "
           "safety sphere must still block the move";
}

// --- Sphere-vs-AABB geometry regressions ------------------------------------------------------
//
// The two tests below each isolate one specific bug that a voxel-center-only safety model would
// miss, independently of one another: the first is purely about geometry (a diagonal voxel whose
// *center* lies beyond the radius but whose *volume* still clips the sphere); the second is
// purely about which position the sphere is centered at (the real drone is not always exactly at
// its own voxel's center). Both use a fully-Empty grid (via fillAllEmptyIn) with exactly one
// Occupied voxel, so with the fix the algorithm should refuse the only candidate move, find
// nothing left to map, and finish immediately with no movement at all -- with either original bug
// still present, it issues an Advance into the unsafe candidate instead.

TEST(MappingAlgorithm, SphereVsAabbRejectsDiagonalCornerVoxelBeyondOldCenterDistanceCheck) {
    // radius=8cm, resolution=10cm: the diagonal (+x,+y) neighbor of the candidate (3,2,2) sits at
    // center-to-center distance sqrt(10^2+10^2)=14.14cm (> 8cm -- invisible to a check that skips
    // any voxel whose *center* lies beyond the radius) but its nearest AABB corner is only
    // sqrt(5^2+5^2)=7.07cm from the candidate's own center (<= 8cm) -- a real drone sphere does
    // clip that corner, so it must still block the move.
    constexpr long kVoxelsPerAxis = 6;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    map->set(vc(4, 3, 2), VoxelOccupancy::Occupied); // diagonal corner neighbor of candidate (3,2,2)

    const auto drone = customDroneConfig(/*radius_cm=*/8.0, 90.0, 10.0, 10.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    const DroneState state{vc(2, 2, 2), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "the only candidate move's safety sphere clips the diagonal Occupied voxel's corner "
           "and must be rejected, even though that voxel's center lies beyond the drone's radius";
    EXPECT_EQ(command.status, AlgorithmStatus::Finished)
        << "with the unsafe candidate correctly rejected and nothing else left to map, the "
           "algorithm should finish immediately instead of moving into it";
}

TEST(MappingAlgorithm, SafetyCheckPreservesRealIntraVoxelOffsetInsteadOfAssumingVoxelCenter) {
    // The real drone sits at x=28cm, 3cm off its own voxel's (2,2,2) nominal center (25cm) --
    // e.g. the mission's initial position was never grid-aligned. Advancing one resolution step
    // (+10cm) truly lands the drone at x=38cm, not at the target voxel (3,2,2)'s own nominal
    // center (35cm). An Occupied voxel at (4,2,2) (x in [40,50)) is 2cm from the true future
    // position (38cm) but 5cm from the naive, un-offset candidate center (35cm) -- with a 4cm
    // drone radius, only evaluating the real, offset-preserved position (38cm) detects the
    // collision; evaluating the naive voxel center (35cm) would wrongly call it safe.
    constexpr long kVoxelsPerAxis = 6;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    map->set(vc(4, 2, 2), VoxelOccupancy::Occupied); // 2cm from the true future position, 5cm from the naive one

    const auto drone = customDroneConfig(/*radius_cm=*/4.0, 90.0, 10.0, 10.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    const DroneState state{pos(28.0, 25.0, 25.0), Orientation{}, 0}; // 3cm off voxel (2,2,2)'s center in x

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "the candidate move's true future position (offset-preserved) collides with the "
           "Occupied voxel, even though the naive (un-offset) voxel center would not";
    EXPECT_EQ(command.status, AlgorithmStatus::Finished)
        << "with the unsafe candidate correctly rejected and nothing else left to map, the "
           "algorithm should finish immediately instead of moving into it";
}

// --- Movement path safety across multiple steps -------------------------------------------------

TEST(MappingAlgorithm, MovementNeverPassesThroughAnUnvalidatedVoxelWhenLaneIsBlocked) {
    // A regression test: the candidate immediately past an obstruction must never be reached by
    // a single straight-line movement that skips over the obstruction itself.
    const auto config = customGridConfig(7, 10.0);
    auto map = freshMap(config);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    map->set(vc(3, 3, 3), VoxelOccupancy::Empty); // start
    map->set(vc(4, 3, 3), VoxelOccupancy::Occupied); // obstruction directly ahead
    map->set(vc(5, 3, 3), VoxelOccupancy::Empty); // tempting (but unreachable in one step) cell

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 50.0, 50.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    const DroneState state{vc(3, 3, 3), Orientation{}, 0};

    runUntilDoneVerifyingPathSafety(algorithm, state, *map, /*radius_cm=*/1.0, 30);
    // expectPositionSafe() (invoked on every position-changing step inside the helper) already
    // fails the test via EXPECT_EQ/ASSERT_TRUE if any unsafe voxel was ever entered.
}

TEST(MappingAlgorithm, MovementStaysSafeAcrossAFullSweepLayerBoundary) {
    // Regression test: once an entire xy sweep layer (z=0) is fully Empty and traversed, the
    // sweep cursor wraps back to the layer's starting row while stepping up to z=1 - a transition
    // that is *not* a single face-adjacent grid step (it jumps across most of the y-range). Every
    // z=1 voxel is left Unmapped (unsafe) except the one exact wrap-destination cell, which is
    // Empty. A naive single straight-line movement to that destination would pass straight
    // through the unsafe z=1 voxels it skips over (including the very first one reached by the
    // Elevate alone); this test fails on the unsafe intermediate position if that happens.
    constexpr long kVoxelsPerAxis = 5;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    for (long ix = 0; ix < kVoxelsPerAxis; ++ix) {
        for (long iy = 0; iy < kVoxelsPerAxis; ++iy) {
            map->set(vc(ix, iy, 0), VoxelOccupancy::Empty); // the entire z=0 layer.
        }
    }
    // The boustrophedon sweep starting at (0,0,0) ends its z=0 layer at (4,4,0) (5 rows, x
    // direction flips each row); the wrap candidate is therefore (4, 0, 1). Leave only that one
    // z=1 cell Empty; every other z=1 cell (including (4,4,1), reached by Elevate alone, and
    // (4,1,1)/(4,2,1)/(4,3,1) along the skipped straight line) stays Unmapped.
    map->set(vc(4, 0, 1), VoxelOccupancy::Empty);

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, /*max_advance_cm=*/10.0,
                                          /*max_elevate_cm=*/10.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    const DroneState state{vc(0, 0, 0), Orientation{}, 0};

    ASSERT_NO_THROW(runUntilDoneVerifyingPathSafety(algorithm, state, *map, /*radius_cm=*/1.0, 200));
}

TEST(MappingAlgorithm, MovementPathRemainsSafeThroughComplexPartiallyMappedMap) {
    // A larger, partially-mapped map with an obstacle wall and a single-voxel corridor gap, plus
    // one remaining Unmapped pocket beyond the wall. Verifies every movement step (sweep moves
    // through the pre-mapped Empty region, then frontier BFS routes through the corridor) lands
    // only on Empty, in-bounds, sphere-safe voxels.
    constexpr long kVoxelsPerAxis = 9;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };

    // A wall at x=4 across the whole y/z range, except a one-voxel-wide corridor at y=4.
    for (long iy = 0; iy < kVoxelsPerAxis; ++iy) {
        for (long iz = 0; iz < kVoxelsPerAxis; ++iz) {
            if (iy == 4) {
                continue; // corridor gap
            }
            map->set(vc(4, iy, iz), VoxelOccupancy::Occupied);
        }
    }
    map->set(vc(7, 4, 4), VoxelOccupancy::Unmapped); // pocket beyond the wall, reached via the gap

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 100.0), drone, *map});
    const DroneState state{vc(1, 4, 4), Orientation{}, 0};

    runUntilDoneVerifyingPathSafety(algorithm, state, *map, /*radius_cm=*/1.0, 500);
}

// --- Map boundaries -------------------------------------------------------------------------

TEST(MappingAlgorithm, NeverMovesDroneOutsideMapBoundaries) {
    // A small, tight map (3 voxels per axis) means the drone is always close to an edge, so any
    // boundary-overrun bug in the sweep/frontier logic is exercised quickly.
    constexpr long kVoxelsPerAxis = 3;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    const auto drone = customDroneConfig(1.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    // expectPositionSafe() (called for every position-changing step) asserts isInBounds(), which
    // checks against output_map_.getMapConfig().boundaries - exactly what this test targets.
    runUntilDoneVerifyingPathSafety(algorithm, state, *map, /*radius_cm=*/1.0, 200);
}

// --- Non-zero map offset --------------------------------------------------------------------

TEST(MappingAlgorithm, NonZeroOffsetForwardOccupiedVoxelBlocksMovement) {
    const auto config = customGridConfig(5, 10.0, /*offset_x_cm=*/120.0, /*offset_y_cm=*/340.0,
                                          /*offset_z_cm=*/-50.0);
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 1, 0, 0), VoxelOccupancy::Occupied);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "with a non-zero/negative map offset, the forward voxel's world position must still "
           "resolve to the Occupied cell and block movement";
}

TEST(MappingAlgorithm, NonZeroOffsetScanOrientationStillPointsAtForwardTarget) {
    const auto config = customGridConfig(5, 10.0, 120.0, 340.0, -50.0);
    auto map = freshMap(config); // forward voxel stays Unmapped.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_NEAR(command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0)
        << "the forward target is due east of the drone regardless of map offset; the relative "
           "scan orientation must reflect that, not the offset";
}

// --- Non-default resolution ------------------------------------------------------------------

TEST(MappingAlgorithm, NonDefaultResolutionChunkedAdvanceSumsToResolutionStep) {
    constexpr double kResolutionCm = 25.0;
    const auto config = customGridConfig(5, kResolutionCm);
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 1, 0, 0), VoxelOccupancy::Empty);
    const auto drone = customDroneConfig(1.0, 30.0, /*max_advance_cm=*/10.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    double total_cm = 0.0;
    for (int i = 0; i < 10; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value());
        ASSERT_EQ(command.movement->type, MovementCommandType::Advance);
        EXPECT_LE(command.movement->distance.force_numerical_value_in(cm), 10.0 + 1e-6);
        total_cm += command.movement->distance.force_numerical_value_in(cm);
        state = applyMovement(state, command.movement);
        if (total_cm >= kResolutionCm - 1e-6) {
            break;
        }
    }

    EXPECT_NEAR(total_cm, kResolutionCm, 1e-6)
        << "chunked advances must sum to exactly one resolution step (25cm), not the default 10cm";
}

TEST(MappingAlgorithm, NonDefaultResolutionSafetyStillBlocksOccupiedForwardVoxel) {
    const auto config = customGridConfig(5, 25.0);
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 1, 0, 0), VoxelOccupancy::Occupied);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_FALSE(command.movement.has_value())
        << "safety must still block an Occupied forward voxel under a 25cm resolution";
}

// --- Different movement limits -----------------------------------------------------------------

TEST(MappingAlgorithm, DifferentMovementLimitsAreEachRespectedIndividually) {
    const auto config = customGridConfig(5, 10.0);
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 1, 0, 0), VoxelOccupancy::Empty);
    // Tight, mutually-different limits on every movement type.
    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/15.0,
                                          /*max_advance_cm=*/3.0, /*max_elevate_cm=*/2.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), drone, *map});

    DroneState state{voxelCenterIn(config, 0, 0, 0),
                      Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 0};
    for (int i = 0; i < 30; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        if (!command.movement.has_value()) {
            break;
        }
        switch (command.movement->type) {
            case MovementCommandType::Rotate:
                EXPECT_LE(command.movement->angle.force_numerical_value_in(deg), 15.0 + 1e-6);
                break;
            case MovementCommandType::Advance:
                EXPECT_LE(command.movement->distance.force_numerical_value_in(cm), 3.0 + 1e-6);
                break;
            case MovementCommandType::Elevate:
                EXPECT_LE(std::fabs(command.movement->distance.force_numerical_value_in(cm)), 2.0 + 1e-6);
                break;
            case MovementCommandType::Hover:
                break;
        }
        state = applyMovement(state, command.movement);
    }
}

// --- Frontier BFS behavior -----------------------------------------------------------------

TEST(MappingAlgorithm, FrontierBfsFindsSafeFrontierAndTargetsTheCorrectVoxelThroughACorridor) {
    // Same wall-with-corridor layout as the path-safety test above, but this test focuses on the
    // *targeting* behavior: once the algorithm reaches a valid frontier near the pocket, the scan
    // it issues must point at the pocket voxel specifically.
    constexpr long kVoxelsPerAxis = 9;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };

    for (long iy = 0; iy < kVoxelsPerAxis; ++iy) {
        for (long iz = 0; iz < kVoxelsPerAxis; ++iz) {
            if (iy == 4) {
                continue;
            }
            map->set(vc(4, iy, iz), VoxelOccupancy::Occupied);
        }
    }
    const VoxelOccupancy initial_pocket = VoxelOccupancy::Unmapped;
    map->set(vc(7, 4, 4), initial_pocket);

    const auto drone = customDroneConfig(1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 100.0), drone, *map});
    DroneState state{vc(1, 4, 4), Orientation{}, 0};

    const Position3D pocket_center = vc(7, 4, 4);
    bool found_targeted_scan = false;
    for (int i = 0; i < 500 && !found_targeted_scan; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_NE(command.status, AlgorithmStatus::FinishedWithUnmappableVoxels)
            << "the pocket is reachable through the corridor and within lidar range; it must not "
               "be classified unmappable";
        // A single MappingStepCommand may now carry both a movement and a scan; per spec the
        // movement executes first and the scan is issued from the resulting state, so the scan
        // must be validated against the POST-movement position/heading, not the pre-movement
        // `state` that was passed into nextStep().
        const DroneState post_movement_state = applyMovement(state, command.movement);
        if (command.scan_orientation.has_value()) {
            // Recompute the absolute world direction the scan implies and compare it to the
            // direction toward the pocket: MockLidar adds scan_orientation to gps_.heading() to
            // get the absolute beam direction (see relativeScanOrientation's contract).
            const double dx = (pocket_center.x - post_movement_state.position.x).force_numerical_value_in(cm);
            const double dy = (pocket_center.y - post_movement_state.position.y).force_numerical_value_in(cm);
            const double distance_cm = std::sqrt(dx * dx + dy * dy);
            if (distance_cm > customLidarConfig(5.0, 100.0).z_max.force_numerical_value_in(cm)) {
                // Not yet close enough to the pocket for this scan to plausibly target it;
                // keep driving.
                state = post_movement_state;
                state.step_index += 1;
                continue;
            }
            const double world_horizontal_deg = std::atan2(dy, dx) * 180.0 / M_PI;
            const double absolute_scan_deg =
                post_movement_state.heading.horizontal.force_numerical_value_in(deg) +
                command.scan_orientation->horizontal.force_numerical_value_in(deg);
            const double diff_deg = std::fmod(absolute_scan_deg - world_horizontal_deg + 540.0, 360.0) - 180.0;
            if (std::fabs(diff_deg) < 5.0) {
                found_targeted_scan = true;
                continue;
            }
        }
        state = post_movement_state;
        state.step_index += 1;
    }

    EXPECT_TRUE(found_targeted_scan)
        << "expected the algorithm to eventually scan from a frontier near the pocket, with the "
           "scan orientation pointing at the pocket voxel";
}

// --- Frontier BFS adjacency: all 6 face directions, including -z -------------------------------

TEST(MappingAlgorithm, FrontierBfsCanDescendInZToReachATarget) {
    // A single 1x1x3 vertical column: the only neighbors any cell could have are directly above
    // (+z) or below (-z) it, since x and y each have just one valid index (any +-x/+-y neighbor
    // is out of the map's bounds). The drone starts at the top (z=2); a target two voxels below
    // it is out of lidar range from the start, so reaching it requires the frontier BFS to first
    // explore *downward* into the middle cell (z=1), which is the only cell within range of the
    // target. A BFS whose face-direction set is missing -z could never visit that middle cell --
    // there is no other path down this column -- and would wrongly give up with
    // FinishedWithUnmappableVoxels on the very first call instead of proposing a descending move.
    constexpr double kResolutionCm = 10.0;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], kResolutionCm * x_extent[cm], 0.0 * y_extent[cm],
                      kResolutionCm * y_extent[cm], 0.0 * z_extent[cm], 3.0 * kResolutionCm * z_extent[cm]},
        Position3D{},
        kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 0, 0, 2), VoxelOccupancy::Empty);   // start
    map->set(voxelCenterIn(config, 0, 0, 1), VoxelOccupancy::Empty);   // only reachable via -z
    map->set(voxelCenterIn(config, 0, 0, 0), VoxelOccupancy::Unmapped); // target

    // z_min=5cm/z_max=15cm: the target is 20cm from the start (out of range, forcing movement)
    // but only 10cm from the middle cell (in range).
    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, /*max_advance_cm=*/10.0,
                                          /*max_elevate_cm=*/10.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{
        missionConfig(), customLidarConfig(/*z_min=*/5.0, /*z_max=*/15.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 0, 2), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_EQ(command.status, AlgorithmStatus::Working)
        << "the target is reachable by descending one voxel in z, the only direction available in "
           "this single-column map; the algorithm must not give up with FinishedWithUnmappableVoxels";
    ASSERT_TRUE(command.movement.has_value())
        << "expected a movement command toward the only reachable frontier (one voxel below)";
    EXPECT_EQ(command.movement->type, MovementCommandType::Elevate)
        << "the only way to make progress in this vertical column is to descend (-z)";
}

// --- Lidar z_min: targets too close to a frontier are not scannable from it ---------------------

TEST(MappingAlgorithm, TargetTooCloseToOnlyPossibleFrontierIsNeverResolvedDueToZMin) {
    // A 3x3x3 grid (max possible distance between any two cells is the corner distance,
    // sqrt(3)*10 ~= 17.3cm) with z_min set above that distance: every cell in the grid is too
    // close to every other cell to ever be a valid frontier for it under the z_min rule, so the
    // single Unmapped target voxel can never be resolved by a scan.
    constexpr long kVoxelsPerAxis = 3;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    map->set(voxelCenterIn(config, 1, 1, 1), VoxelOccupancy::Unmapped); // the only target, dead center

    const auto drone = customDroneConfig(1.0, 30.0, 5.0, 5.0);
    // z_min (20cm) exceeds every possible distance within this 3-voxel-per-axis grid.
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{
        missionConfig(), customLidarConfig(/*z_min_cm=*/20.0, /*z_max_cm=*/100.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 1, 1), Orientation{}, 0};

    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_EQ(status, AlgorithmStatus::FinishedWithUnmappableVoxels)
        << "the target's only possible frontiers are all closer than z_min, so it must remain "
           "unresolved";
}

// --- Lidar z_max: targets too far from a frontier are not scannable from it ---------------------

TEST(MappingAlgorithm, TargetTooFarFromEveryPossibleFrontierIsNeverResolvedDueToZMax) {
    // z_max (8cm) is smaller than the map's own resolution (10cm), so even the closest possible
    // frontier - a face-adjacent cell - is already farther than z_max from the target. No
    // frontier anywhere in the grid can ever be valid for this target.
    constexpr long kVoxelsPerAxis = 3;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    map->set(voxelCenterIn(config, 1, 1, 1), VoxelOccupancy::Unmapped);

    const auto drone = customDroneConfig(1.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{
        missionConfig(), customLidarConfig(/*z_min_cm=*/1.0, /*z_max_cm=*/8.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 1, 1), Orientation{}, 0};

    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_EQ(status, AlgorithmStatus::FinishedWithUnmappableVoxels)
        << "z_max is smaller than the resolution, so no frontier can ever be close enough to the "
           "target; it must remain unresolved";
}

// --- Finished with only Empty and Occupied voxels (no Unmapped/PotentiallyOccupied) -------------

TEST(MappingAlgorithm, FinishedWhenMapHasOnlyEmptyAndOccupiedVoxels) {
    constexpr long kVoxelsPerAxis = 3;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    // Sprinkle in some Occupied voxels (not just Empty) while leaving the rest of the map
    // reachable; none of these should prevent Finished, since Occupied is fully-resolved data.
    map->set(voxelCenterIn(config, 2, 2, 2), VoxelOccupancy::Occupied);
    map->set(voxelCenterIn(config, 0, 2, 0), VoxelOccupancy::Occupied);
    map->set(voxelCenterIn(config, 2, 0, 2), VoxelOccupancy::Occupied);

    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), lidarConfig(), droneConfig(), *map});
    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    const AlgorithmStatus status = runUntilDone(algorithm, state, 500);

    EXPECT_EQ(status, AlgorithmStatus::Finished)
        << "a map with only Empty and Occupied voxels (no Unmapped/PotentiallyOccupied) must be "
           "considered fully mapped, even though it isn't all-Empty";
}

// --- Line-of-sight blocking rule (Occupied only, not PotentiallyOccupied) ---------------------

TEST(MappingAlgorithm, PotentiallyOccupiedVoxelOnLineOfSightDoesNotBlockTargeting) {
    // The start cell's only Sweep-phase neighbor (+x) is a confirmed Occupied wall, so Sweep's
    // retry-counted scan branch never applies to it (Occupied is not target-occupancy) and the
    // very first nextStep() call deterministically falls through to a Frontier-phase detour --
    // independent of the Sweep retry budget (kMaxSweepScanAttempts), which is unrelated to this
    // test and must not leak into it via a hardcoded call count.
    //
    // The actual case under test lives along +y instead: a target two voxels away that is only
    // visible from the start cell straight through one intervening voxel (no alternate viewing
    // angle exists, since x is walled off and z has no extent). That intervening voxel is
    // PotentiallyOccupied (not a confirmed wall) -- per spec, only a confirmed Occupied voxel may
    // block a line-of-sight scan attempt. A bug that also treats PotentiallyOccupied as blocking
    // would report the target unreachable (FinishedWithUnmappableVoxels) on this very first call,
    // instead of issuing a scan toward it.
    constexpr double kResolutionCm = 10.0;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], 2.0 * kResolutionCm * x_extent[cm], 0.0 * y_extent[cm],
                      3.0 * kResolutionCm * y_extent[cm], 0.0 * z_extent[cm], 1.0 * kResolutionCm * z_extent[cm]},
        Position3D{},
        kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    map->set(voxelCenterIn(config, 0, 0, 0), VoxelOccupancy::Empty);               // start
    map->set(voxelCenterIn(config, 1, 0, 0), VoxelOccupancy::Occupied);            // +x wall
    map->set(voxelCenterIn(config, 0, 1, 0), VoxelOccupancy::PotentiallyOccupied); // obstruction
    map->set(voxelCenterIn(config, 0, 2, 0), VoxelOccupancy::Unmapped);            // target

    // z_min=15cm excludes the obstruction itself (10cm away) from being a viable target in its
    // own right, isolating its role to a pure line-of-sight blocker; z_max=100cm comfortably
    // covers the target at 20cm.
    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{
        missionConfig(), customLidarConfig(/*z_min=*/15.0, /*z_max=*/100.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    EXPECT_EQ(command.status, AlgorithmStatus::Working)
        << "the target two voxels away is only visible through a PotentiallyOccupied voxel; since "
           "that is not a confirmed wall, the algorithm must still be able to target it via a scan "
           "rather than giving up with FinishedWithUnmappableVoxels (no other viewing angle exists: "
           "+x is walled off and z has no extent)";
    EXPECT_TRUE(command.scan_orientation.has_value())
        << "expected a scan command targeting the cell beyond the PotentiallyOccupied voxel";
}

// --- Sweep boustrophedon direction alternation ------------------------------------------------

TEST(MappingAlgorithm, SweepRowsAlternateXDirectionAcrossRows) {
    // A fully-Empty 3x3x1 grid: with nothing Unmapped/PotentiallyOccupied anywhere, the algorithm
    // never needs to scan or detour through Frontier, so its movement is governed purely by the
    // Sweep cursor -- letting this test observe the cursor's actual row-to-row x-direction
    // without any Frontier-phase fallback masking a broken cursor (Frontier's independent BFS
    // would still reach every Empty cell regardless of Sweep's order, which is what lets this bug
    // hide in plan tests that only check final completeness/safety, not the visited-cell order).
    //
    // Every cell here is already known Empty+safe, so Sweep's multi-cell batching merges the
    // straight run within each row into a single Advance command instead of one per voxel (the
    // row's own pivot-in step stays a separate single-cell move, since batching only extends a
    // pure in-lane step, never a pivot) -- the loop below therefore records every voxel a
    // movement *crosses*, not merely the position after it, so the boustrophedon order can still
    // be checked voxel-by-voxel regardless of how many of them one MappingStepCommand's movement
    // now covers.
    constexpr double kResolutionCm = 10.0;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], 3.0 * kResolutionCm * x_extent[cm], 0.0 * y_extent[cm],
                      3.0 * kResolutionCm * y_extent[cm], 0.0 * z_extent[cm], 1.0 * kResolutionCm * z_extent[cm]},
        Position3D{},
        kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < 3; ++ix) {
        for (long iy = 0; iy < 3; ++iy) {
            map->set(voxelCenterIn(config, ix, iy, 0), VoxelOccupancy::Empty);
        }
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    // Record the (ix, iy) grid cell each time it changes, tracing the cursor's actual path --
    // sampling every resolution-sized step of a movement (not just its destination), so a batched
    // multi-voxel Advance still yields one entry per voxel crossed.
    std::vector<std::pair<long, long>> visited{{0, 0}};
    for (int i = 0; i < 60; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        if (command.status != AlgorithmStatus::Working) {
            break;
        }
        const DroneState next_state = applyMovement(state, command.movement);
        if (command.movement.has_value() &&
            (command.movement->type == MovementCommandType::Advance ||
             command.movement->type == MovementCommandType::Elevate)) {
            const double distance_cm = std::fabs(command.movement->distance.force_numerical_value_in(cm));
            const long num_steps = std::max(1L, std::lround(distance_cm / kResolutionCm));
            for (long s = 1; s <= num_steps; ++s) {
                const double t = static_cast<double>(s) / static_cast<double>(num_steps);
                const Position3D sample{
                    state.position.x + t * (next_state.position.x - state.position.x),
                    state.position.y + t * (next_state.position.y - state.position.y),
                    state.position.z + t * (next_state.position.z - state.position.z),
                };
                const TestVoxelIndex idx = toIndexIn(config, sample);
                if (visited.back() != std::make_pair(idx.ix, idx.iy)) {
                    visited.push_back({idx.ix, idx.iy});
                }
            }
        }
        state = next_state;
        state.step_index += 1;
    }

    // Row y=0 must be fully traversed in increasing x order: (0,0),(1,0),(2,0).
    ASSERT_GE(visited.size(), 4u) << "expected at least the full first row plus one row-2 cell";
    EXPECT_EQ(visited[1], std::make_pair(1L, 0L));
    EXPECT_EQ(visited[2], std::make_pair(2L, 0L));

    // After the row-end pivot to (2,1), row y=1 must proceed in *decreasing* x order: the very
    // next cell visited must be (1,1), not (2,2) -- which is what a cursor stuck going only in
    // +x (the bug: missing the direction flip) would produce instead, since it would immediately
    // re-exhaust the lane at x=2 and pivot to the next row again without ever moving to x=1 or
    // x=0 in row 1.
    ASSERT_GE(visited.size(), 6u) << "expected the cursor to continue past the row-1 pivot cell";
    EXPECT_EQ(visited[3], std::make_pair(2L, 1L)) << "row-end pivot must keep x and step to y=1";
    EXPECT_EQ(visited[4], std::make_pair(1L, 1L))
        << "row 1 must proceed in the opposite x-direction from row 0 (boustrophedon pattern); "
           "jumping straight to (2,2) would skip x=1 and x=0 in this row entirely";
    EXPECT_EQ(visited[5], std::make_pair(0L, 1L));
}

// --- Elevation / 3D movement -----------------------------------------------------------------

TEST(MappingAlgorithm, ElevateCommandsRespectMaxElevateWhileClimbingToATarget) {
    // The whole grid is pre-mapped Empty except the topmost-center voxel, which can only be
    // resolved by a drone that has climbed up through the z-axis - exercising Elevate commands.
    constexpr long kVoxelsPerAxis = 3;
    constexpr double kResolutionCm = 20.0;
    const auto config = customGridConfig(kVoxelsPerAxis, kResolutionCm);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    map->set(voxelCenterIn(config, 1, 1, 2), VoxelOccupancy::Unmapped); // top-center pocket

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 100.0, /*max_elevate_cm=*/6.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 100.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    bool saw_elevate = false;
    double max_observed_elevate_cm = 0.0;
    AlgorithmStatus status = AlgorithmStatus::Working;
    for (int i = 0; i < 500 && status == AlgorithmStatus::Working; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        status = command.status;
        if (command.movement.has_value() && command.movement->type == MovementCommandType::Elevate) {
            saw_elevate = true;
            const double distance_cm = std::fabs(command.movement->distance.force_numerical_value_in(cm));
            max_observed_elevate_cm = std::max(max_observed_elevate_cm, distance_cm);
            EXPECT_LE(distance_cm, 6.0 + 1e-6) << "Elevate command exceeded max_elevate";
        }
        state = applyMovement(state, command.movement);
        state.step_index += 1;
    }

    EXPECT_TRUE(saw_elevate) << "reaching the top-z pocket requires at least one Elevate command";
    EXPECT_LE(max_observed_elevate_cm, 6.0 + 1e-6);
}

// --- PotentiallyOccupied treated as a real, targetable unresolved voxel -------------------------

TEST(MappingAlgorithm, PotentiallyOccupiedVoxelGetsATargetedScanFromAValidFrontier) {
    // Stronger than merely checking status != Finished: this verifies the algorithm actually
    // finds a safe frontier for the PotentiallyOccupied voxel and aims a scan at it specifically.
    constexpr long kVoxelsPerAxis = 5;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    map->set(voxelCenterIn(config, 3, 3, 3), VoxelOccupancy::PotentiallyOccupied);

    const auto drone = customDroneConfig(1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    const Position3D target_center = voxelCenterIn(config, 3, 3, 3);
    bool found_targeted_scan = false;
    for (int i = 0; i < 500 && !found_targeted_scan; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_NE(command.status, AlgorithmStatus::Finished)
            << "a reachable PotentiallyOccupied voxel must not be treated as already mapped";
        // A single MappingStepCommand may now carry both a movement and a scan; per spec the
        // movement executes first and the scan is issued from the resulting state, so the scan
        // must be validated against the POST-movement position/heading, not the pre-movement
        // `state` that was passed into nextStep().
        const DroneState post_movement_state = applyMovement(state, command.movement);
        if (command.scan_orientation.has_value()) {
            const double dx = (target_center.x - post_movement_state.position.x).force_numerical_value_in(cm);
            const double dy = (target_center.y - post_movement_state.position.y).force_numerical_value_in(cm);
            const double dz = (target_center.z - post_movement_state.position.z).force_numerical_value_in(cm);
            const double distance_cm = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance_cm >= 5.0 - 1e-6 && distance_cm <= 50.0 + 1e-6) {
                const double horizontal_dist_cm = std::sqrt(dx * dx + dy * dy);
                const double world_horizontal_deg = std::atan2(dy, dx) * 180.0 / M_PI;
                const double world_altitude_deg = std::atan2(dz, horizontal_dist_cm) * 180.0 / M_PI;
                const double absolute_scan_horizontal_deg =
                    post_movement_state.heading.horizontal.force_numerical_value_in(deg) +
                    command.scan_orientation->horizontal.force_numerical_value_in(deg);
                const double absolute_scan_altitude_deg =
                    post_movement_state.heading.altitude.force_numerical_value_in(deg) +
                    command.scan_orientation->altitude.force_numerical_value_in(deg);
                const double horizontal_diff =
                    std::fmod(absolute_scan_horizontal_deg - world_horizontal_deg + 540.0, 360.0) - 180.0;
                if (std::fabs(horizontal_diff) < 5.0 &&
                    std::fabs(absolute_scan_altitude_deg - world_altitude_deg) < 5.0) {
                    found_targeted_scan = true;
                    continue;
                }
            }
        }
        state = post_movement_state;
        state.step_index += 1;
    }

    EXPECT_TRUE(found_targeted_scan)
        << "expected a scan from a valid frontier (within [z_min, z_max] of the target) aimed "
           "specifically at the PotentiallyOccupied voxel";
}

// --- Small / tight maps ----------------------------------------------------------------------

TEST(MappingAlgorithm, TinyMapWithRadiusComparableToResolutionFinishesWithoutDeadlock) {
    // A 2-voxel-per-axis map where the drone's safety-sphere radius is close
    // to the resolution.
    constexpr long kVoxelsPerAxis = 2;
    constexpr double kResolutionCm = 10.0;
    const auto config = customGridConfig(kVoxelsPerAxis, kResolutionCm);
    auto map = freshMap(config);
    const auto drone = customDroneConfig(/*radius_cm=*/9.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    AlgorithmStatus status = AlgorithmStatus::Working;
    ASSERT_NO_THROW(status = runUntilDone(algorithm, state, 500));
    EXPECT_NE(status, AlgorithmStatus::Working)
        << "the algorithm must terminate (Finished or FinishedWithUnmappableVoxels) within 500 "
           "steps on a tiny 2x2x2 map, not deadlock";
}

TEST(MappingAlgorithm, SingleVoxelMapTerminatesWithoutDeadlock) {
    // The most extreme version of the tiny-map risk above: a 1x1x1 map has no neighbor voxel at
    // all, so there is no possible frontier or sweep target other than the drone's own (Unmapped)
    // starting cell. A bug that assumes at least one other voxel always exists (e.g. an
    // unconditional sweep-direction lookup) could infinite-loop or throw instead of cleanly
    // reporting FinishedWithUnmappableVoxels.
    const auto config = customGridConfig(/*voxels_per_axis=*/1, /*resolution_cm=*/10.0);
    auto map = freshMap(config);
    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 30.0, 5.0, 5.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});

    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};
    AlgorithmStatus status = AlgorithmStatus::Working;
    ASSERT_NO_THROW(status = runUntilDone(algorithm, state, 500));
    EXPECT_NE(status, AlgorithmStatus::Working)
        << "the algorithm must terminate within 500 steps on a 1x1x1 map with no possible "
           "frontier, not deadlock";
}

// --- Movement+scan pipelining and movement-chunk merging (Frontier phase) ----------------------
//
// All tests below use a 1-cell-wide corridor/column (x and z, or x and y, pinned to a single
// cell) with the drone starting at the *far end* of the one non-trivial axis. That start position
// makes Sweep's own lawnmower cursor wrap past the entire sweep volume on the very first
// nextStep() call (mirrored from FrontierBfsCanDescendInZToReachATarget's z-column trick), so
// `phase` becomes Frontier immediately and every command in these tests is produced by Frontier's
// own BFS + buildMovementQueue, not by Sweep (whose own multi-cell batching is covered separately,
// further below).

namespace {

// A corridor 7 voxels long in y (x and z pinned to a single cell), resolution 10cm. Every cell
// from iy=1..6 is pre-mapped Empty; iy=0 is left Unmapped (the default) as the scan target.
// z_min=5cm/z_max=15cm admit only the immediate (10cm) neighbor as a valid scanning frontier, so
// BFS must walk the other five cells (iy=5..1) before a scan of iy=0 becomes possible -- a
// deterministic 5-step straight run, matching the "5 x 10cm, max_advance>=50cm -> single 50cm
// Advance" example from the task description.
std::unique_ptr<Map3DImpl> yCorridorMap(const MapConfig& config, long y_cells) {
    auto map = freshMap(config);
    for (long iy = 1; iy < y_cells; ++iy) {
        map->set(voxelCenterIn(config, 0, iy, 0), VoxelOccupancy::Empty);
    }
    return map;
}

MapConfig yCorridorConfig(long y_cells, double resolution_cm) {
    return MapConfig{
        MappingBounds{0.0 * x_extent[cm], resolution_cm * x_extent[cm], 0.0 * y_extent[cm],
                      static_cast<double>(y_cells) * resolution_cm * y_extent[cm], 0.0 * z_extent[cm],
                      resolution_cm * z_extent[cm]},
        Position3D{}, resolution_cm * isq::length[cm]};
}

} // namespace

TEST(MappingAlgorithm, FrontierCombinesMergedAdvanceRunWithScanInOneStep) {
    constexpr double kResolutionCm = 10.0;
    constexpr long kYCells = 7;
    const auto config = yCorridorConfig(kYCells, kResolutionCm);
    const auto map = yCorridorMap(config, kYCells);

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/50.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 6, 0), Orientation{}, 0};

    const MappingStepCommand rotate_cmd = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(rotate_cmd.movement.has_value());
    EXPECT_EQ(rotate_cmd.movement->type, MovementCommandType::Rotate)
        << "must rotate to face -y (toward the corridor) before advancing into it";
    EXPECT_FALSE(rotate_cmd.scan_orientation.has_value())
        << "the rotate does not reach the frontier by itself; it must not carry the scan";

    const DroneState after_rotate = applyMovement(state, rotate_cmd.movement);
    const MappingStepCommand advance_cmd = algorithm.nextStep(after_rotate, nullptr);
    ASSERT_TRUE(advance_cmd.movement.has_value());
    EXPECT_EQ(advance_cmd.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(advance_cmd.movement->distance.force_numerical_value_in(cm), 50.0, 1e-6)
        << "five consecutive 10cm steps with max_advance=50cm must merge into one 50cm Advance, "
           "not five separate 10cm Advances";
    ASSERT_TRUE(advance_cmd.scan_orientation.has_value())
        << "the scan for the iy=0 target must be attached to this Advance: it is the command "
           "that actually reaches the frontier (iy=1), so movement and scan combine into one "
           "MappingStepCommand / mission step";

    // The scan orientation must reflect the drone's PREDICTED POST-movement heading (now facing
    // -y): from the frontier, the target sits directly ahead along the new heading, so the
    // relative angle must be ~0deg. (This corridor is collinear -- the target sits on the same
    // straight line the whole run travels -- so it cannot by itself distinguish a PRE- vs
    // POST-movement *position* bug, only a stale-heading one; see
    // FrontierCombinedScanOrientationReflectsPostMovementPositionNotPreMovementPosition below for
    // a bent-path scenario where position alone changes the correct answer.)
    EXPECT_NEAR(advance_cmd.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0);
    EXPECT_NEAR(advance_cmd.scan_orientation->altitude.force_numerical_value_in(deg), 0.0, 1.0);

    // Old (pre-pipelining) behavior needed 1 rotate + 5 single-voxel Advances + 1 separate scan
    // = 7 mission steps to reach this same point; the pipelined version needs exactly 2 (as
    // demonstrated by the two nextStep() calls above already having produced both the movement
    // and the scan).
}

TEST(MappingAlgorithm, FrontierAttachesScanOnlyToTheFinalChunkOfAMergedAdvanceRun) {
    // Same corridor as above, but max_advance=20cm forces the merged 50cm run to chunk into
    // 20+20+10cm. Only the LAST chunk (the one that actually reaches the frontier) may carry the
    // scan; the earlier chunks must be movement-only, and safety/path validation is unaffected
    // since every underlying voxel was already validated by frontierBfs before any chunking.
    constexpr double kResolutionCm = 10.0;
    constexpr long kYCells = 7;
    const auto config = yCorridorConfig(kYCells, kResolutionCm);
    const auto map = yCorridorMap(config, kYCells);

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/20.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 6, 0), Orientation{}, 0};

    const MappingStepCommand rotate_cmd = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(rotate_cmd.movement.has_value());
    ASSERT_EQ(rotate_cmd.movement->type, MovementCommandType::Rotate);
    EXPECT_FALSE(rotate_cmd.scan_orientation.has_value());
    state = applyMovement(state, rotate_cmd.movement);

    std::vector<double> advance_distances_cm;
    std::vector<bool> had_scan;
    for (int i = 0; i < 5; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value());
        ASSERT_EQ(command.movement->type, MovementCommandType::Advance);
        EXPECT_LE(command.movement->distance.force_numerical_value_in(cm), 20.0 + 1e-6)
            << "each chunk must respect max_advance even though the underlying run is merged";
        advance_distances_cm.push_back(command.movement->distance.force_numerical_value_in(cm));
        had_scan.push_back(command.scan_orientation.has_value());
        state = applyMovement(state, command.movement);
        if (command.scan_orientation.has_value()) {
            break;
        }
    }

    ASSERT_EQ(advance_distances_cm.size(), 3u)
        << "50cm at max_advance=20cm must chunk into exactly three Advances (20+20+10), matching "
           "the task's 230cm/max_advance=100cm -> 100+100+30 chunking example";
    EXPECT_NEAR(advance_distances_cm[0], 20.0, 1e-6);
    EXPECT_NEAR(advance_distances_cm[1], 20.0, 1e-6);
    EXPECT_NEAR(advance_distances_cm[2], 10.0, 1e-6);
    EXPECT_FALSE(had_scan[0]) << "the first chunk must be movement-only";
    EXPECT_FALSE(had_scan[1]) << "the second chunk must be movement-only";
    EXPECT_TRUE(had_scan[2]) << "the scan must attach to the third (final) chunk only";
}

TEST(MappingAlgorithm, FrontierCombinesMergedElevateRunWithScanInOneStep) {
    // Vertical counterpart of the Advance tests above (1x1x7 column, x and y pinned to a single
    // cell), exercising Elevate merging specifically. With no rotation needed, the entire 5-step
    // descent collapses into a single Elevate command that is simultaneously the first and the
    // last command in the queue, so it must carry the movement and the scan together on the very
    // first nextStep() call.
    constexpr double kResolutionCm = 10.0;
    constexpr long kZCells = 7;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], kResolutionCm * x_extent[cm], 0.0 * y_extent[cm],
                      kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      static_cast<double>(kZCells) * kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long iz = 1; iz < kZCells; ++iz) {
        map->set(voxelCenterIn(config, 0, 0, iz), VoxelOccupancy::Empty);
    }
    // iz = 0 stays Unmapped (the default) -- the scan target.

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, /*max_advance_cm=*/50.0,
                                          /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 0, 6), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, MovementCommandType::Elevate);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), -50.0, 1e-6)
        << "five consecutive -10cm steps with max_elevate=50cm must merge into one -50cm Elevate";
    ASSERT_TRUE(command.scan_orientation.has_value())
        << "with no rotation needed and the whole run merged into a single command, that command "
           "is simultaneously the first and the last, and must carry both the movement and the "
           "scan for the iz=0 target in one mission step (old behavior needed 6: 5 Elevates + 1 "
           "separate scan)";
    EXPECT_NEAR(command.scan_orientation->altitude.force_numerical_value_in(deg), -90.0, 1.0)
        << "from the frontier (iz=1), the target directly below must be scanned at ~-90deg "
           "altitude, computed against the drone's still-level heading -- Elevate never changes "
           "horizontal/altitude heading";
}

TEST(MappingAlgorithm, ResolvePendingScanWorksAfterACombinedMovementAndScanStep) {
    // Regression test for the pending-scan bookkeeping introduced by movement+scan combination:
    // once a combined command has been returned, the *next* nextStep() call must resolve it
    // exactly like a separately-issued scan always did (see
    // OutputMapIsAuthoritativeOverPreviousScanRequest) -- i.e. if the map now shows the target
    // resolved, the algorithm must not re-target it or misreport the mission as unmappable.
    constexpr double kResolutionCm = 10.0;
    constexpr long kYCells = 7;
    const auto config = yCorridorConfig(kYCells, kResolutionCm);
    const auto map = yCorridorMap(config, kYCells);

    const auto drone = customDroneConfig(1.0, 90.0, /*max_advance_cm=*/50.0, 50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 6, 0), Orientation{}, 0};

    const MappingStepCommand rotate_cmd = algorithm.nextStep(state, nullptr);
    state = applyMovement(state, rotate_cmd.movement);
    const MappingStepCommand combined = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(combined.movement.has_value());
    ASSERT_TRUE(combined.scan_orientation.has_value());
    state = applyMovement(state, combined.movement);

    // Simulate what DroneControlImpl + ScanResultToVoxels would have done after that scan: the
    // only remaining Unmapped voxel (iy=0) resolves to a wall.
    map->set(voxelCenterIn(config, 0, 0, 0), VoxelOccupancy::Occupied);

    const MappingStepCommand next = algorithm.nextStep(state, nullptr);
    EXPECT_EQ(next.status, AlgorithmStatus::Finished)
        << "the target resolved to Occupied and no other voxel in this 1x7x1 corridor is "
           "unresolved; the algorithm must recognize the combined step's scan as having "
           "succeeded and finish, instead of re-targeting iy=0 or giving up as unmappable";
}

// --- Sweep-phase movement+scan pipelining -------------------------------------------------------
//
// "scan candidate A -> move into now-known-safe A + scan candidate B -> move into now-known-safe
// B + scan candidate C -> ..." -- one candidate ahead at a time. These tests deliberately keep
// only a single further cell known Empty at any point (B, then C), so movement batching (covered
// on its own, further below) never has anything to extend into and this scan-pipeline mechanism
// can be observed in isolation. A 4x1x1 lane keeps Sweep's own lawnmower cursor moving in a
// single, entirely predictable straight line along +x, matching the drone's initial heading
// exactly, so no Rotate ever appears and every command below is either a plain scan or a
// plain/combined Advance.

TEST(MappingAlgorithm, SweepPipelinesScanOfNextCandidateOntoTheMoveIntoTheCurrentOne) {
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 4;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix) { return voxelCenterIn(config, ix, 0, 0); };
    // ix=0 (the start cell) is exempt from needing a known occupancy; ix=1..3 all start Unmapped
    // (the default) and get resolved to Empty by the test, mirroring what DroneControlImpl +
    // ScanResultToVoxels would do with each scan's result.

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/50.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    DroneState state{vc(0), Orientation{}, 0};

    // Step 1: candidate A (ix=1) is Unmapped -- Sweep must scan it directly, with no movement (it
    // is not yet known safe to enter -- "never move into an unknown/unverified voxel").
    const MappingStepCommand scan_a = algorithm.nextStep(state, nullptr);
    EXPECT_FALSE(scan_a.movement.has_value());
    ASSERT_TRUE(scan_a.scan_orientation.has_value());
    map->set(vc(1), VoxelOccupancy::Empty);

    // Step 2: A is now known Empty and safe -- Sweep must move into it. B (ix=2) is still
    // Unmapped and face-adjacent to A, so scanning it from A's position is semantically valid
    // once the move completes; that scan must be attached to this same move.
    const MappingStepCommand move_a_scan_b = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(move_a_scan_b.movement.has_value());
    EXPECT_EQ(move_a_scan_b.movement->type, MovementCommandType::Advance);
    ASSERT_TRUE(move_a_scan_b.scan_orientation.has_value())
        << "the scan of the next candidate (ix=2) must be attached to the move into the current "
           "one (ix=1), combining them into a single mission step instead of two";
    EXPECT_NEAR(move_a_scan_b.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0)
        << "from ix=1, ix=2 is directly ahead along the never-rotated heading";
    state = applyMovement(state, move_a_scan_b.movement);
    map->set(vc(2), VoxelOccupancy::Empty);

    // Step 3: the pipeline must continue one candidate at a time -- move into B (ix=2), with the
    // scan of C (ix=3) attached -- proving this is a genuine chain, not a one-off special case.
    const MappingStepCommand move_b_scan_c = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(move_b_scan_c.movement.has_value());
    EXPECT_EQ(move_b_scan_c.movement->type, MovementCommandType::Advance);
    ASSERT_TRUE(move_b_scan_c.scan_orientation.has_value())
        << "the pipeline must continue: the scan of C (ix=3) must attach to the move into B";
}

TEST(MappingAlgorithm, SweepDoesNotAttachAScanWhenTheCandidateAfterTheBatchIsNotAValidTarget) {
    // If the candidate immediately after the batched movement's occupancy is already known
    // (Occupied here) at the time the batch is built, attaching a scan for it would not be
    // "semantically valid from the post-movement state" -- there is nothing left to learn from
    // scanning an already-resolved voxel. Note that with multi-cell batching, an *already-known
    // Empty* next cell (B, C below) does NOT stay unscanned-and-unvisited the way it would have
    // pre-batching: it is simply absorbed into the same movement batch as A, since it passes the
    // exact same Empty+safe check A itself did. Only a candidate that fails that check (Occupied
    // D here) actually stops the batch and is then evaluated for the scan-pipeline.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 5;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix) { return voxelCenterIn(config, ix, 0, 0); };
    // B (ix=2) and C (ix=3) are already known Empty before A is ever scanned -- both must be
    // absorbed into the batch alongside A. D (ix=4) is already known Occupied -- not a valid
    // scan target, and not Empty either, so it stops the batch without joining it.
    map->set(vc(2), VoxelOccupancy::Empty);
    map->set(vc(3), VoxelOccupancy::Empty);
    map->set(vc(4), VoxelOccupancy::Occupied);

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/50.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    DroneState state{vc(0), Orientation{}, 0};

    const MappingStepCommand scan_a = algorithm.nextStep(state, nullptr);
    ASSERT_FALSE(scan_a.movement.has_value());
    ASSERT_TRUE(scan_a.scan_orientation.has_value());
    map->set(vc(1), VoxelOccupancy::Empty); // A resolves; B and C were already Empty.

    const MappingStepCommand move_abc = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(move_abc.movement.has_value());
    EXPECT_EQ(move_abc.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(move_abc.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "A, B, and C are all Empty+safe by the time this move is built, so batching must "
           "merge all three into one 30cm Advance instead of stopping after just A";
    EXPECT_FALSE(move_abc.scan_orientation.has_value())
        << "D (ix=4) -- the candidate right after the batch -- is already known Occupied, not a "
           "valid scan target; the batch's final movement must not manufacture a scan for it";
}

// --- Stronger post-movement scan-orientation check (Frontier) ------------------------------------

TEST(MappingAlgorithm, FrontierCombinedScanOrientationReflectsPostMovementPositionNotPreMovementPosition) {
    // Regression for a blind spot in the corridor tests above: those use a target collinear with
    // the whole travel line, so a bug that computed the scan from the PRE-movement position
    // instead of the correct POST-movement one would produce the *same* angle either way and
    // slip through undetected. Here the drone travels along row y=4 while the target sits one
    // row south, at the far (x=0) end: the bearing to the target from the START (x=4) differs
    // from the bearing from the FRONTIER (x=0, right next to the target) by roughly 76 degrees,
    // so using the wrong position visibly produces the wrong angle.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 5;
    constexpr long kYCells = 5;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], static_cast<double>(kYCells) * kResolutionCm * y_extent[cm],
                      0.0 * z_extent[cm], kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix, long iy) { return voxelCenterIn(config, ix, iy, 0); };

    for (long ix = 0; ix < kXCells; ++ix) {
        map->set(vc(ix, 4), VoxelOccupancy::Empty); // the travel corridor.
    }
    for (long ix = 1; ix < kXCells; ++ix) {
        map->set(vc(ix, 3), VoxelOccupancy::Occupied); // wall, except directly below x=0.
    }
    // vc(0, 3) stays Unmapped (the default) -- the scan target; (0,4) is its only in-range Empty
    // neighbor (z_max below excludes the ~14.14cm diagonal to (1,4)), so BFS has exactly one
    // (frontier, target) pair available: no ambiguity about which route gets taken.

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/180.0,
                                          /*max_advance_cm=*/40.0, /*max_elevate_cm=*/40.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 12.0), drone, *map});
    // Start at the grid's last lawnmower cell so Sweep's own cursor immediately wraps past the
    // entire volume on the very first call (see the column tests above) and every command below
    // comes from Frontier.
    DroneState state{vc(4, 4), Orientation{}, 0};

    const MappingStepCommand rotate_cmd = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(rotate_cmd.movement.has_value());
    ASSERT_EQ(rotate_cmd.movement->type, MovementCommandType::Rotate);
    EXPECT_FALSE(rotate_cmd.scan_orientation.has_value());
    state = applyMovement(state, rotate_cmd.movement);

    const DroneState pre_movement_state = state;
    const MappingStepCommand advance_cmd = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(advance_cmd.movement.has_value());
    ASSERT_EQ(advance_cmd.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(advance_cmd.movement->distance.force_numerical_value_in(cm), 40.0, 1e-6)
        << "four consecutive 10cm steps with max_advance=40cm must merge into one 40cm Advance";
    ASSERT_TRUE(advance_cmd.scan_orientation.has_value());
    const DroneState post_movement_state = applyMovement(state, advance_cmd.movement);

    const Position3D target_center = vc(0, 3);
    const auto worldBearingDeg = [](const Position3D& from, const Position3D& to) {
        return std::atan2((to.y - from.y).force_numerical_value_in(cm),
                           (to.x - from.x).force_numerical_value_in(cm)) *
               180.0 / M_PI;
    };
    const double bearing_from_pre_deg = worldBearingDeg(pre_movement_state.position, target_center);
    const double bearing_from_post_deg = worldBearingDeg(post_movement_state.position, target_center);
    ASSERT_GE(std::fabs(std::remainder(bearing_from_post_deg - bearing_from_pre_deg, 360.0)), 30.0)
        << "sanity check on the geometry itself: pre- and post-movement bearings to the target "
           "must clearly differ, or this test cannot distinguish the two";

    // MockLidar adds scan_orientation to the heading *at scan time* (post-movement; the Advance
    // itself does not change heading) to get the absolute beam direction.
    const double implied_absolute_scan_deg =
        post_movement_state.heading.horizontal.force_numerical_value_in(deg) +
        advance_cmd.scan_orientation->horizontal.force_numerical_value_in(deg);

    EXPECT_NEAR(std::remainder(implied_absolute_scan_deg - bearing_from_post_deg, 360.0), 0.0, 5.0)
        << "the scan orientation must point at the target from the POST-movement position";
    EXPECT_GE(std::fabs(std::remainder(implied_absolute_scan_deg - bearing_from_pre_deg, 360.0)), 15.0)
        << "the scan orientation must NOT match the bearing computed from the PRE-movement "
           "position -- a bug that used the drone's position before this Advance would produce "
           "this instead";
}

// --- Movement merging stops at a direction change (Frontier) -------------------------------------

TEST(MappingAlgorithm, FrontierMergedAdvanceStopsAtADirectionChangeThenResumesMergingOnTheNextLeg) {
    // Regression for buildMovementQueue's run-grouping: an L-shaped BFS path must produce two
    // separately-merged Advance commands (one per straight leg), never a single Advance summing
    // across the corner, and never falling back to one Advance per resolution-sized grid step
    // either. The horizontal leg here is 3 steps (30cm) and the vertical leg is 2 steps (20cm) --
    // deliberately different lengths so a bug that merged across the corner (which would total
    // 50cm) is distinguishable, by inspection of the two reported distances, from correct
    // per-leg merging.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 5;
    constexpr long kYCells = 5;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], static_cast<double>(kYCells) * kResolutionCm * y_extent[cm],
                      0.0 * z_extent[cm], kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix, long iy) { return voxelCenterIn(config, ix, iy, 0); };

    // The L-shaped corridor: horizontal leg (row y=4, x=4 down to x=1), then a turn south into a
    // vertical leg (column x=1, y=4 down to y=2).
    for (long ix = 1; ix <= 4; ++ix) {
        map->set(vc(ix, 4), VoxelOccupancy::Empty);
    }
    map->set(vc(1, 3), VoxelOccupancy::Empty);
    map->set(vc(1, 2), VoxelOccupancy::Empty);
    // vc(1, 1) stays Unmapped (the default) -- the scan target, reachable only from vc(1, 2).

    // Wall off every other face-neighbor of the corridor/target so the only connected route is
    // the L-shape above, and no cell along it accidentally sees a nearer stray Unmapped voxel
    // (everything not explicitly touched here defaults to Unmapped).
    for (const auto& [ix, iy] : std::vector<std::pair<long, long>>{
             {4, 3}, {3, 3}, {2, 3}, {0, 4}, {0, 3}, {0, 2}, {2, 2}, {0, 1}, {2, 1}, {1, 0}}) {
        map->set(vc(ix, iy), VoxelOccupancy::Occupied);
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/180.0,
                                          /*max_advance_cm=*/30.0, /*max_elevate_cm=*/30.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 12.0), drone, *map});
    DroneState state{vc(4, 4), Orientation{}, 0};

    std::vector<double> advance_distances_cm;
    bool saw_scan = false;
    for (int i = 0; i < 6 && !saw_scan; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value());
        if (command.movement->type == MovementCommandType::Advance) {
            advance_distances_cm.push_back(command.movement->distance.force_numerical_value_in(cm));
        }
        if (command.scan_orientation.has_value()) {
            saw_scan = true;
        }
        state = applyMovement(state, command.movement);
    }

    ASSERT_TRUE(saw_scan) << "expected the drone to reach the frontier and scan the target";
    ASSERT_EQ(advance_distances_cm.size(), 2u)
        << "the L-shaped path must produce exactly two merged Advances (one per straight leg), "
           "neither one 50cm Advance across the corner nor five separate 10cm Advances";
    EXPECT_NEAR(advance_distances_cm[0], 30.0, 1e-6) << "the first (horizontal) leg is 3 steps";
    EXPECT_NEAR(advance_distances_cm[1], 20.0, 1e-6) << "the second (vertical) leg is 2 steps";
}

// --- Sweep multi-cell batching -------------------------------------------------------------------
//
// "A Empty/safe, B Empty/safe, C Empty/safe, D Unmapped" -> one batched path {A, B, C} handed to
// buildMovementQueue(), instead of processing A, then B, then C on separate Sweep iterations.

TEST(MappingAlgorithm, SweepBatchesConsecutiveCandidatesIntoOneMergedAdvanceAndAttachesScanOfTheFirstUnresolvedCandidate) {
    // The canonical batching example from the task description: A, B, C already Empty+safe; D
    // still Unmapped. A single sweepStep() call must batch {A, B, C} into one path handed to
    // buildMovementQueue() (merging it into one 30cm Advance, since max_advance easily covers it)
    // and, per the existing movement+scan pipeline, attach a scan of D to that same command,
    // since D is face-adjacent to C (the batch's final cell) and still needs one.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 5;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix) { return voxelCenterIn(config, ix, 0, 0); };
    map->set(vc(1), VoxelOccupancy::Empty); // A
    map->set(vc(2), VoxelOccupancy::Empty); // B
    map->set(vc(3), VoxelOccupancy::Empty); // C
    // vc(4) stays Unmapped (the default) -- D, the scan target.

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/50.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    const DroneState state{vc(0), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "A, B, and C (3 * 10cm) must merge into a single 30cm Advance from one sweepStep() "
           "call, instead of three separate 10cm moves across three calls";
    ASSERT_TRUE(command.scan_orientation.has_value())
        << "D is Unmapped and face-adjacent to C (the batch's final cell); its scan must attach "
           "to this same combined command";
    EXPECT_NEAR(command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0)
        << "D sits directly ahead of C along the unchanged (never-rotated) heading";
}

TEST(MappingAlgorithm, SweepBatchedLegIsChunkedWhenItExceedsMaxAdvance) {
    // Same A/B/C/D setup as above, but max_advance=12cm forces the merged 30cm batch to chunk
    // into 12+12+6cm, exercising buildMovementQueue's existing chunking logic on top of the new
    // batched path. Only the final chunk -- the one that actually reaches C -- may carry D's scan.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 5;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    const auto vc = [&](long ix) { return voxelCenterIn(config, ix, 0, 0); };
    map->set(vc(1), VoxelOccupancy::Empty);
    map->set(vc(2), VoxelOccupancy::Empty);
    map->set(vc(3), VoxelOccupancy::Empty);

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/12.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    DroneState state{vc(0), Orientation{}, 0};

    std::vector<double> advance_distances_cm;
    std::vector<bool> had_scan;
    for (int i = 0; i < 5; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value());
        ASSERT_EQ(command.movement->type, MovementCommandType::Advance);
        EXPECT_LE(command.movement->distance.force_numerical_value_in(cm), 12.0 + 1e-6)
            << "each chunk must respect max_advance even though the underlying batch is merged";
        advance_distances_cm.push_back(command.movement->distance.force_numerical_value_in(cm));
        had_scan.push_back(command.scan_orientation.has_value());
        state = applyMovement(state, command.movement);
        if (command.scan_orientation.has_value()) {
            break;
        }
    }

    ASSERT_EQ(advance_distances_cm.size(), 3u)
        << "30cm at max_advance=12cm must chunk into exactly three Advances (12+12+6)";
    EXPECT_NEAR(advance_distances_cm[0], 12.0, 1e-6);
    EXPECT_NEAR(advance_distances_cm[1], 12.0, 1e-6);
    EXPECT_NEAR(advance_distances_cm[2], 6.0, 1e-6);
    EXPECT_FALSE(had_scan[0]) << "the first chunk must be movement-only";
    EXPECT_FALSE(had_scan[1]) << "the second chunk must be movement-only";
    EXPECT_TRUE(had_scan[2]) << "the scan of D must attach to the third (final) chunk only";
}

TEST(MappingAlgorithm, SweepBatchingStopsBeforeACandidateThatFailsTheSafetyCheckEvenThoughItsOwnVoxelIsEmpty) {
    // A candidate can be Empty itself yet still unsafe to enter, if an Occupied voxel sits within
    // the drone's safety sphere without being directly in the path (see the SphereSafety* tests
    // earlier in this file). Batching must apply that exact same isSafeVoxel() check to every
    // candidate it considers, not just occupancy -- so it must stop right after B, leaving the
    // Empty-but-unsafe C for the (unaffected) state machine to skip via its Frontier detour on a
    // later call.
    constexpr long kVoxelsPerAxis = 9;
    const auto config = customGridConfig(kVoxelsPerAxis, 10.0);
    auto map = freshMap(config);
    fillAllEmptyIn(*map, config, kVoxelsPerAxis);
    const auto vc = [&](long ix, long iy, long iz) { return voxelCenterIn(config, ix, iy, iz); };
    // Path: start=(0,4,4), A=(1,4,4), B=(2,4,4), C=(3,4,4) -- all Empty from fillAllEmptyIn. The
    // blocker sits diagonally ahead of C (closest-AABB-corner distance ~7.07cm, within the 12cm
    // safety radius below) but far enough from B (~15.8cm) to leave it unaffected.
    map->set(vc(4, 5, 4), VoxelOccupancy::Occupied);

    const auto drone = customDroneConfig(/*radius_cm=*/12.0, /*max_rotate_deg=*/90.0,
                                          /*max_advance_cm=*/50.0, /*max_elevate_cm=*/50.0);
    MappingAlgorithmImpl algorithm(
        MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 15.0), drone, *map});
    const DroneState state{vc(0, 4, 4), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 20.0, 1e-6)
        << "the batch must stop after A and B (2 * 10cm); C is Empty but fails the safety check "
           "and must not be included, even though occupancy alone would have allowed it";
}

TEST(MappingAlgorithm, SweepBatchingDoesNotCrossARowPivotOrLayerTransition) {
    // Two full rows (y=0 and y=1), both entirely Empty. Even though the cell immediately after
    // the lane boundary (the row-1 pivot destination) is itself Empty+safe too, batching must
    // stop exactly at the lane boundary in this checkpoint -- it may not fold a row-pivot or
    // layer transition into the same batch/movement leg.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 4;
    constexpr long kYCells = 2;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], static_cast<double>(kYCells) * kResolutionCm * y_extent[cm],
                      0.0 * z_extent[cm], kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < kXCells; ++ix) {
        for (long iy = 0; iy < kYCells; ++iy) {
            map->set(voxelCenterIn(config, ix, iy, 0), VoxelOccupancy::Empty);
        }
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, /*max_advance_cm=*/100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    const DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    const MappingStepCommand command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "row y=0 has 3 cells past the start (3 * 10cm); the batch must stop there instead of "
           "folding the row-1 pivot destination (also Empty+safe) into the same movement";
    EXPECT_FALSE(command.scan_orientation.has_value())
        << "the row-1 pivot destination is already known Empty, so no scan is warranted either";
}

TEST(MappingAlgorithm, SweepBatchingCursorBookkeepingVisitsEveryCellExactlyOnceWithNoSkipsOrRepeats) {
    // A fully-Empty 4x2x1 grid, entirely traversed via batched movement legs. Regardless of how
    // many voxels a single MappingStepCommand's movement now covers, the sweep cursor's
    // consume-exactly-what-was-batched bookkeeping must still visit every cell exactly once: a
    // skipped cell would show up missing from the visited set (or force the mission to finish
    // with unmapped voxels remaining), and a doubly-processed one would show up as a duplicate.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 4;
    constexpr long kYCells = 2;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], static_cast<double>(kYCells) * kResolutionCm * y_extent[cm],
                      0.0 * z_extent[cm], kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < kXCells; ++ix) {
        for (long iy = 0; iy < kYCells; ++iy) {
            map->set(voxelCenterIn(config, ix, iy, 0), VoxelOccupancy::Empty);
        }
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, 100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    // Sampled the same way as SweepRowsAlternateXDirectionAcrossRows above: every resolution-sized
    // step of a movement is recorded, not just its destination, so a batched multi-voxel Advance
    // still yields one entry per voxel crossed.
    std::vector<std::pair<long, long>> visited{{0, 0}};
    AlgorithmStatus status = AlgorithmStatus::Working;
    for (int i = 0; i < 30 && status == AlgorithmStatus::Working; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        status = command.status;
        const DroneState next_state = applyMovement(state, command.movement);
        if (command.movement.has_value() &&
            (command.movement->type == MovementCommandType::Advance ||
             command.movement->type == MovementCommandType::Elevate)) {
            const double distance_cm = std::fabs(command.movement->distance.force_numerical_value_in(cm));
            const long num_steps = std::max(1L, std::lround(distance_cm / kResolutionCm));
            for (long s = 1; s <= num_steps; ++s) {
                const double t = static_cast<double>(s) / static_cast<double>(num_steps);
                const Position3D sample{
                    state.position.x + t * (next_state.position.x - state.position.x),
                    state.position.y + t * (next_state.position.y - state.position.y),
                    state.position.z + t * (next_state.position.z - state.position.z),
                };
                const TestVoxelIndex idx = toIndexIn(config, sample);
                if (visited.back() != std::make_pair(idx.ix, idx.iy)) {
                    visited.push_back({idx.ix, idx.iy});
                }
            }
        }
        state = next_state;
        state.step_index += 1;
    }

    EXPECT_EQ(status, AlgorithmStatus::Finished)
        << "every cell in this fully-Empty grid is already resolved; the mission must finish "
           "cleanly, which a skipped cell (misreported as unmappable) or a stuck cursor would "
           "prevent";

    const std::set<std::pair<long, long>> unique_visited(visited.begin(), visited.end());
    EXPECT_EQ(unique_visited.size(), visited.size())
        << "the same cell was recorded more than once -- the sweep cursor processed a candidate "
           "twice";
    EXPECT_EQ(unique_visited.size(), static_cast<std::size_t>(kXCells * kYCells))
        << "expected all " << (kXCells * kYCells) << " cells to be visited exactly once; got "
        << unique_visited.size();
}

TEST(MappingAlgorithm, SweepBatchingReducesStepCountForALongStraightLaneWhileFinishingCorrectly) {
    // An 8-cell single row, already fully known Empty+safe: pre-batching, traversing it required
    // one Advance-bearing nextStep() call per voxel (7, since the start cell needs no move).
    // Batching must now cover the entire row in a single merged Advance, while still finishing
    // the mission correctly (every cell was already Empty, so nothing else is left to map).
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 8;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < kXCells; ++ix) {
        map->set(voxelCenterIn(config, ix, 0, 0), VoxelOccupancy::Empty);
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, 90.0, /*max_advance_cm=*/100.0, 100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    int advance_command_count = 0;
    AlgorithmStatus status = AlgorithmStatus::Working;
    for (int i = 0; i < 30 && status == AlgorithmStatus::Working; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        status = command.status;
        if (command.movement.has_value() && command.movement->type == MovementCommandType::Advance) {
            ++advance_command_count;
        }
        state = applyMovement(state, command.movement);
        state.step_index += 1;
    }

    EXPECT_EQ(status, AlgorithmStatus::Finished);
    EXPECT_EQ(advance_command_count, 1)
        << "the whole 7-voxel row must merge into a single Advance command (batched, then merged "
           "by buildMovementQueue) instead of the 7 separate single-voxel Advances pre-batching "
           "would have needed";
}

TEST(MappingAlgorithm, SweepBatchingTreatsARowPivotAsItsOwnStepThenBatchesTheRestOfTheNewLaneOnTheFollowingCall) {
    // Regression for a scope leak: the batch always starts at `candidate` and extends using
    // sweep_dir_x, so if `candidate` is *itself* a row-pivot destination (advanceSweepCandidate()
    // already flipped sweep_dir_x to produce it), naively extending from it would fold the pivot
    // move together with several cells of the *new* row into one batch/movement leg -- crossing
    // exactly the row/lane transition this checkpoint excludes. Two full rows, both entirely
    // Empty: row y=0 (3 cells past the start) batches normally; the row-1 pivot destination must
    // then be its own single 10cm step; only the *following* call may batch the remaining 3 cells
    // of row y=1.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 4;
    constexpr long kYCells = 2;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], static_cast<double>(kYCells) * kResolutionCm * y_extent[cm],
                      0.0 * z_extent[cm], kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < kXCells; ++ix) {
        for (long iy = 0; iy < kYCells; ++iy) {
            map->set(voxelCenterIn(config, ix, iy, 0), VoxelOccupancy::Empty);
        }
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/180.0,
                                          /*max_advance_cm=*/100.0, /*max_elevate_cm=*/100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    // Command 1: row y=0's batch (3 cells past the start).
    const MappingStepCommand row0_batch = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(row0_batch.movement.has_value());
    ASSERT_EQ(row0_batch.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(row0_batch.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "row y=0 has 3 cells past the start";
    state = applyMovement(state, row0_batch.movement);

    // Command 2: rotate to face the row-1 pivot destination.
    const MappingStepCommand pivot_rotate = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(pivot_rotate.movement.has_value());
    ASSERT_EQ(pivot_rotate.movement->type, MovementCommandType::Rotate);
    state = applyMovement(state, pivot_rotate.movement);

    // Command 3: the pivot move itself -- exactly one 10cm Advance, NOT merged with any of row
    // y=1's remaining cells (which are also Empty+safe, and would extend a naively-built batch).
    const MappingStepCommand pivot_move = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(pivot_move.movement.has_value());
    ASSERT_EQ(pivot_move.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(pivot_move.movement->distance.force_numerical_value_in(cm), 10.0, 1e-6)
        << "the row-1 pivot destination must be reached by its own single 10cm step, not folded "
           "together with the rest of row y=1 into one larger batch";
    EXPECT_FALSE(pivot_move.scan_orientation.has_value());
    state = applyMovement(state, pivot_move.movement);

    // Command 4: rotate to face back along row y=1 (opposite direction from row y=0).
    const MappingStepCommand row1_rotate = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(row1_rotate.movement.has_value());
    ASSERT_EQ(row1_rotate.movement->type, MovementCommandType::Rotate);
    state = applyMovement(state, row1_rotate.movement);

    // Command 5: NOW -- on the call after the drone has actually arrived at the pivot
    // destination -- the remaining 3 cells of row y=1 must batch together normally.
    const MappingStepCommand row1_rest_batch = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(row1_rest_batch.movement.has_value());
    ASSERT_EQ(row1_rest_batch.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(row1_rest_batch.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "once actually at the pivot destination, the remaining 3 cells of row y=1 must batch "
           "into a single 30cm Advance just like row y=0 did";
}

TEST(MappingAlgorithm, SweepBatchingTreatsALayerTransitionAsItsOwnStepThenBatchesTheNewLayerOnTheFollowingCall) {
    // Same regression as above, but for a genuine layer transition (z changes) rather than a
    // row pivot (y changes): a 4x1x2 volume (y pinned to a single cell, so every row-boundary
    // pivot immediately becomes a layer transition too -- see advanceSweepCandidate()) with both
    // z-layers entirely Empty. Layer z=0's row batches normally; the layer-transition destination
    // (an Elevate, face-adjacent to where layer 0 ended) must then be its own single step; only
    // the *following* call may batch the remaining cells of layer z=1.
    constexpr double kResolutionCm = 10.0;
    constexpr long kXCells = 4;
    constexpr long kZCells = 2;
    const MapConfig config{
        MappingBounds{0.0 * x_extent[cm], static_cast<double>(kXCells) * kResolutionCm * x_extent[cm],
                      0.0 * y_extent[cm], kResolutionCm * y_extent[cm], 0.0 * z_extent[cm],
                      static_cast<double>(kZCells) * kResolutionCm * z_extent[cm]},
        Position3D{}, kResolutionCm * isq::length[cm]};
    auto map = freshMap(config);
    for (long ix = 0; ix < kXCells; ++ix) {
        for (long iz = 0; iz < kZCells; ++iz) {
            map->set(voxelCenterIn(config, ix, 0, iz), VoxelOccupancy::Empty);
        }
    }

    const auto drone = customDroneConfig(/*radius_cm=*/1.0, /*max_rotate_deg=*/180.0,
                                          /*max_advance_cm=*/100.0, /*max_elevate_cm=*/100.0);
    MappingAlgorithmImpl algorithm(MappingAlgorithmDependencies{missionConfig(), customLidarConfig(5.0, 50.0), drone, *map});
    DroneState state{voxelCenterIn(config, 0, 0, 0), Orientation{}, 0};

    // Command 1: layer z=0's row batch (3 cells past the start).
    const MappingStepCommand layer0_batch = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(layer0_batch.movement.has_value());
    ASSERT_EQ(layer0_batch.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(layer0_batch.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6);
    state = applyMovement(state, layer0_batch.movement);

    // Command 2: the layer-transition move itself -- exactly one 10cm Elevate, NOT merged with
    // any of layer z=1's remaining cells (also Empty+safe, and would extend a naively-built
    // batch since they stay face-adjacent along the flipped x-direction from here).
    const MappingStepCommand transition_move = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(transition_move.movement.has_value());
    ASSERT_EQ(transition_move.movement->type, MovementCommandType::Elevate);
    EXPECT_NEAR(std::fabs(transition_move.movement->distance.force_numerical_value_in(cm)), 10.0, 1e-6)
        << "the layer transition must be its own single 10cm Elevate, not folded together with "
           "the rest of layer z=1 into one larger batch";
    EXPECT_FALSE(transition_move.scan_orientation.has_value());
    state = applyMovement(state, transition_move.movement);

    // Command 3: rotate to face back along layer z=1 (opposite x-direction from layer z=0).
    const MappingStepCommand layer1_rotate = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(layer1_rotate.movement.has_value());
    ASSERT_EQ(layer1_rotate.movement->type, MovementCommandType::Rotate);
    state = applyMovement(state, layer1_rotate.movement);

    // Command 4: NOW the remaining 3 cells of layer z=1 must batch together normally.
    const MappingStepCommand layer1_rest_batch = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(layer1_rest_batch.movement.has_value());
    ASSERT_EQ(layer1_rest_batch.movement->type, MovementCommandType::Advance);
    EXPECT_NEAR(layer1_rest_batch.movement->distance.force_numerical_value_in(cm), 30.0, 1e-6)
        << "once actually on layer z=1, its remaining 3 cells must batch into a single 30cm "
           "Advance just like layer z=0's row did";
}
