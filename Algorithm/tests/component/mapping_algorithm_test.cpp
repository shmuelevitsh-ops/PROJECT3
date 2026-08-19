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

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using namespace common;
using namespace common::types;
using namespace Algorithm_322889890_315113738;
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

// Asserts the spec's safety rule directly against the map's public API (atVoxel/isInBounds):
// every voxel overlapping a sphere of `radius_cm` centered at `pos` must be Empty, and `pos`
// itself must be in bounds. Used to verify that movements proposed by the algorithm never end up
// somewhere unsafe - not by inspecting algorithm internals, only by querying the injected map.
void expectPositionSafe(const IMap3D& map, const MapConfig& config, double radius_cm, const Position3D& pos) {
    ASSERT_TRUE(map.isInBounds(pos)) << "drone position is outside the map's configured boundaries";
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const long voxel_radius = static_cast<long>(std::ceil(radius_cm / resolution_cm));
    const TestVoxelIndex idx = toIndexIn(config, pos);

    for (long dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (long dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (long dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                const double offset_distance_cm =
                    std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_cm;
                if (offset_distance_cm > radius_cm) {
                    continue;
                }
                const Position3D neighbor_center =
                    voxelCenterIn(config, idx.ix + dx, idx.iy + dy, idx.iz + dz);
                EXPECT_EQ(map.atVoxel(neighbor_center), VoxelOccupancy::Empty)
                    << "voxel within the drone's safety sphere at offset (" << dx << "," << dy << "," << dz
                    << ") from the drone's position is not Empty";
            }
        }
    }
}

// Drives `algorithm` like runUntilDone(), but additionally asserts - after every step that
// actually changes position - that the new position satisfies the sphere-safety rule against the
// *current* map contents. This verifies the full movement path is safe, not only final
// destinations: a bug that skips a single nextStep() call's destination past an unsafe
// intermediate voxel would be caught here on the very step that proposes it.
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
            expectPositionSafe(map, config, radius_cm, next_state.position);
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
        if (command.scan_orientation.has_value()) {
            // Recompute the absolute world direction the scan implies and compare it to the
            // direction toward the pocket: MockLidar adds scan_orientation to gps_.heading() to
            // get the absolute beam direction (see relativeScanOrientation's contract).
            const double dx = (pocket_center.x - state.position.x).force_numerical_value_in(cm);
            const double dy = (pocket_center.y - state.position.y).force_numerical_value_in(cm);
            const double distance_cm = std::sqrt(dx * dx + dy * dy);
            if (distance_cm > customLidarConfig(5.0, 100.0).z_max.force_numerical_value_in(cm)) {
                // Not yet close enough to the pocket for this scan to plausibly target it;
                // keep driving.
                state = applyMovement(state, command.movement);
                state.step_index += 1;
                continue;
            }
            const double world_horizontal_deg = std::atan2(dy, dx) * 180.0 / M_PI;
            const double absolute_scan_deg =
                state.heading.horizontal.force_numerical_value_in(deg) +
                command.scan_orientation->horizontal.force_numerical_value_in(deg);
            const double diff_deg = std::fmod(absolute_scan_deg - world_horizontal_deg + 540.0, 360.0) - 180.0;
            if (std::fabs(diff_deg) < 5.0) {
                found_targeted_scan = true;
                continue;
            }
        }
        state = applyMovement(state, command.movement);
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

    // Record the (ix, iy) grid cell each time it changes, tracing the cursor's actual path.
    std::vector<std::pair<long, long>> visited{{0, 0}};
    for (int i = 0; i < 60; ++i) {
        const MappingStepCommand command = algorithm.nextStep(state, nullptr);
        if (command.status != AlgorithmStatus::Working) {
            break;
        }
        state = applyMovement(state, command.movement);
        state.step_index += 1;
        const TestVoxelIndex idx = toIndexIn(config, state.position);
        if (visited.back() != std::make_pair(idx.ix, idx.iy)) {
            visited.push_back({idx.ix, idx.iy});
        }
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
        if (command.scan_orientation.has_value()) {
            const double dx = (target_center.x - state.position.x).force_numerical_value_in(cm);
            const double dy = (target_center.y - state.position.y).force_numerical_value_in(cm);
            const double dz = (target_center.z - state.position.z).force_numerical_value_in(cm);
            const double distance_cm = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance_cm >= 5.0 - 1e-6 && distance_cm <= 50.0 + 1e-6) {
                const double horizontal_dist_cm = std::sqrt(dx * dx + dy * dy);
                const double world_horizontal_deg = std::atan2(dy, dx) * 180.0 / M_PI;
                const double world_altitude_deg = std::atan2(dz, horizontal_dist_cm) * 180.0 / M_PI;
                const double absolute_scan_horizontal_deg =
                    state.heading.horizontal.force_numerical_value_in(deg) +
                    command.scan_orientation->horizontal.force_numerical_value_in(deg);
                const double absolute_scan_altitude_deg =
                    state.heading.altitude.force_numerical_value_in(deg) +
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
        state = applyMovement(state, command.movement);
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
