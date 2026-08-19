// Migrated from Project 2 (FILES PROJECT 2/tests/components/scan_result_to_voxels_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespaces/includes only, no other behavioral changes from the Project 2 original.
// Uses the real Simulator-owned Map3DImpl as a concrete IMutableMap3D (test-only
// dependency, same as Algorithm/tests/component's use of Map3DImpl -- see
// PROJ2_TESTS_PLAN.md §6) rather than a mock, since ScanResultToVoxels needs to
// observe actual voxel writes across a beam, not just interaction counts.

#include <MissionControl/ScanResultToVoxels.h>
#include <Simulator/Map3DImpl.h>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

using namespace common;
using namespace common::types;
using namespace MissionControl_322889890_315113738;
using simulator::Map3DImpl;

namespace {

constexpr double kResolutionCm = 10.0;
constexpr double kExtentCm = 200.0;
constexpr int kVoxelsPerAxis = static_cast<int>(kExtentCm / kResolutionCm);

Position3D pos(double x_cm, double y_cm, double z_cm) {
    return Position3D{x_cm * x_extent[cm], y_cm * y_extent[cm], z_cm * z_extent[cm]};
}

// A MapConfig spanning [0, kExtentCm) cm on every axis at kResolutionCm
// resolution, with no offset, so world position == index * resolution.
MapConfig testMapConfig() {
    return MapConfig{
        MappingBounds{
            0.0 * x_extent[cm], kExtentCm * x_extent[cm],
            0.0 * y_extent[cm], kExtentCm * y_extent[cm],
            0.0 * z_extent[cm], kExtentCm * z_extent[cm]},
        Position3D{},
        kResolutionCm * isq::length[cm]};
}

std::shared_ptr<Map3DImpl> makeMap() {
    return std::make_shared<Map3DImpl>(std::make_shared<NpyArray>(), testMapConfig());
}

types::LidarConfigData testLidarConfig() {
    return types::LidarConfigData{20.0 * isq::length[cm], 100.0 * isq::length[cm], 0.0 * isq::length[cm], 0};
}

// Counts how many voxels in the test map's grid currently hold `value`.
int countVoxelsWith(const Map3DImpl& map, VoxelOccupancy value) {
    int count = 0;
    for (int i = 0; i < kVoxelsPerAxis; ++i) {
        for (int j = 0; j < kVoxelsPerAxis; ++j) {
            for (int k = 0; k < kVoxelsPerAxis; ++k) {
                if (map.atVoxel(pos(i * kResolutionCm, j * kResolutionCm, k * kResolutionCm)) == value) {
                    ++count;
                }
            }
        }
    }
    return count;
}

} // namespace

TEST(ScanResultToVoxels, NormalHitMarksOccupiedAndEmptyButNotPotentiallyOccupied) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{50.0 * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, testLidarConfig());

    EXPECT_EQ(map->atVoxel(pos(100.0, 50.0, 50.0)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map->atVoxel(pos(70.0, 50.0, 50.0)), VoxelOccupancy::Empty);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::PotentiallyOccupied), 0);
}

TEST(ScanResultToVoxels, TooCloseHitMarksPotentiallyOccupiedAndNeverOccupied) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{0.0 * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, testLidarConfig());

    EXPECT_EQ(map->atVoxel(scan_origin), VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Occupied), 0);
}

// New — closes a Project 2 mutation-coverage gap (DRO11, see
// PROJ2_TESTS_PLAN.md §8.2). scan_origin sits 50cm outside the map's [0,200)
// x-bound; the hit's endpoint (80cm further along +x, at world x=30) lands
// well inside the map. A normal hit's Occupied endpoint is written via a
// setIfStronger() call separate from markBeamSegment()'s own per-point walk
// (which breaks on the very first out-of-bounds sample -- i.e. scan_origin
// itself -- and so can never reach the endpoint on its own). That separate
// call has its own independent bounds check, so it would still mark the
// endpoint if only the leading scan_origin check were skipped -- this
// isolates that specific check, not just "the beam never reaches the map".
TEST(ScanResultToVoxels, OutOfBoundsScanOriginIsANoOp) {
    auto map = makeMap();
    const Position3D scan_origin = pos(-50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{80.0 * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, testLidarConfig());

    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Occupied), 0)
        << "an out-of-bounds scan_origin must be a complete no-op, even though the hit's "
           "endpoint alone would land inside the map";
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Empty), 0);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::PotentiallyOccupied), 0);
}

TEST(ScanResultToVoxels, MissMarksEmptyAndNeverOccupiedOrPotentiallyOccupied) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{std::numeric_limits<double>::max() * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, testLidarConfig());

    EXPECT_EQ(map->atVoxel(scan_origin), VoxelOccupancy::Empty);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Occupied), 0);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::PotentiallyOccupied), 0);
}

TEST(ScanResultToVoxels, MissMarksEmptyAlongEntireBeamUpToZMax) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{std::numeric_limits<double>::max() * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    // Non-default z_max (testLidarConfig() uses 100cm).
    const types::LidarConfigData lidar_config{20.0 * isq::length[cm], 60.0 * isq::length[cm],
                                              0.0 * isq::length[cm], 0};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, lidar_config);

    // A point partway along the beam (not just the origin) must be Empty.
    EXPECT_EQ(map->atVoxel(pos(80.0, 50.0, 50.0)), VoxelOccupancy::Empty);
    // A point at z_max (distance=60cm from the origin) must also be Empty.
    EXPECT_EQ(map->atVoxel(pos(110.0, 50.0, 50.0)), VoxelOccupancy::Empty);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Occupied), 0);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::PotentiallyOccupied), 0);
}

TEST(ScanResultToVoxels, TooCloseHitMarksPotentiallyOccupiedAlongEntireNearSegmentUpToZMin) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{0.0 * isq::length[cm],
                              Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};
    // Non-default z_min (testLidarConfig() uses 20cm).
    const types::LidarConfigData lidar_config{30.0 * isq::length[cm], 100.0 * isq::length[cm],
                                              0.0 * isq::length[cm], 0};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, lidar_config);

    // A point away from the origin but before z_min must be PotentiallyOccupied.
    EXPECT_EQ(map->atVoxel(pos(65.0, 50.0, 50.0)), VoxelOccupancy::PotentiallyOccupied);
    // A point at z_min (distance=30cm from the origin) must also be PotentiallyOccupied.
    EXPECT_EQ(map->atVoxel(pos(80.0, 50.0, 50.0)), VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(countVoxelsWith(*map, VoxelOccupancy::Occupied), 0);
}

TEST(ScanResultToVoxels, CombinesDroneHeadingAndRelativeBeamAngle) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    // drone_heading (+90deg) and hit.angle (-90deg) must add to 0deg absolute
    // (world +x); neither angle alone would produce that direction (+90deg
    // alone is world +y, -90deg alone is world -y).
    const Orientation drone_heading{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const types::LidarHit hit{50.0 * isq::length[cm],
                              Orientation{-90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {hit}, testLidarConfig());

    EXPECT_EQ(map->atVoxel(pos(100.0, 50.0, 50.0)), VoxelOccupancy::Occupied)
        << "drone_heading (90deg) + hit.angle (-90deg) = 0deg absolute (world +x); "
           "neither angle alone would produce this direction.";
}

TEST(ScanResultToVoxels, ConflictingWritesKeepStrongerOccupiedEvidence) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const Orientation beam_angle{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    // A normal hit at distance=50cm marks pos(100,50,50) Occupied. A second,
    // overlapping miss along the same beam re-walks 0..z_max=100cm and would
    // otherwise mark that same position Empty -- but Occupied evidence is
    // stronger and must not be downgraded, regardless of write order.
    const types::LidarHit normal_hit{50.0 * isq::length[cm], beam_angle};
    const types::LidarHit miss_hit{std::numeric_limits<double>::max() * isq::length[cm], beam_angle};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {normal_hit, miss_hit},
                                   testLidarConfig());

    EXPECT_EQ(map->atVoxel(pos(100.0, 50.0, 50.0)), VoxelOccupancy::Occupied)
        << "Occupied evidence from the normal hit must not be downgraded to Empty by "
           "the overlapping miss along the same beam.";
}

// New — closes a Project 2 mutation-coverage gap (DRO07, see
// PROJ2_TESTS_PLAN.md §8.3). ConflictingWritesKeepStrongerOccupiedEvidence
// above only exercises Occupied-vs-Empty priority; it says nothing about the
// Empty-vs-PotentiallyOccupied ordering underneath it, which is the pair
// DRO07 actually swaps.
TEST(ScanResultToVoxels, EmptyEvidenceOutranksPotentiallyOccupiedRegardlessOfWriteOrder) {
    auto map = makeMap();
    const Position3D scan_origin = pos(50.0, 50.0, 50.0);
    const Orientation drone_heading{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const Orientation beam_angle{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    // A miss marks scan_origin (and the rest of the beam) Empty. A second,
    // overlapping too-close hit (distance=0) along the same beam would mark
    // scan_origin PotentiallyOccupied -- but Empty evidence is stronger and
    // must not be downgraded, regardless of write order.
    const types::LidarHit miss_hit{std::numeric_limits<double>::max() * isq::length[cm], beam_angle};
    const types::LidarHit too_close_hit{0.0 * isq::length[cm], beam_angle};

    ScanResultToVoxels::applyToMap(*map, scan_origin, drone_heading, {miss_hit, too_close_hit},
                                   testLidarConfig());

    EXPECT_EQ(map->atVoxel(scan_origin), VoxelOccupancy::Empty)
        << "Empty evidence from the miss must not be downgraded to PotentiallyOccupied by "
           "the overlapping too-close hit along the same beam.";
}
