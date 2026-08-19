// Migrated from Project 2 (FILES PROJECT 2/tests/components/mock_lidar_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespaces/includes only, no other behavioral changes from the Project 2 original.

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockLidar.h>

#include "mocks/GMockIGPS.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <limits>
#include <memory>

using namespace common;
using namespace common::types;
using namespace simulator;
using ::testing::NiceMock;
using ::testing::Return;

#ifndef DATA_MAPS_DIR
#define DATA_MAPS_DIR "."
#endif

namespace {

// 5x5x5 voxels at 10cm resolution (50cm physical extent); only voxel (4,4,4)
// is occupied, i.e. world cube [40,50) x [40,50) x [40,50) is Occupied, all
// other voxels are Empty.
const std::filesystem::path k_npy =
    std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy";

PhysicalLength res10() {
    return 10.0 * isq::length[cm];
}

Position3D pos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

Orientation headingDeg(double horizontal_deg, double altitude_deg) {
    return Orientation{horizontal_deg * horizontal_angle[deg], altitude_deg * altitude_angle[deg]};
}

// A fixed-value IGPS stub: MockLidar only ever calls position()/heading() to
// place and aim its beams, never any GPS movement/update logic, so a mocked
// GPS keeps these tests isolated from the real MockGPS implementation (a
// separate component with its own behavior) -- only MockLidar's own
// beam-tracing logic is exercised here.
std::unique_ptr<NiceMock<test::GMockIGPS>> makeGps(const Position3D& position, const Orientation& heading) {
    auto gps = std::make_unique<NiceMock<test::GMockIGPS>>();
    ON_CALL(*gps, position()).WillByDefault(Return(position));
    ON_CALL(*gps, heading()).WillByDefault(Return(heading));
    return gps;
}

MappingBounds boundsFromExtentCm(double extent_cm) {
    return MappingBounds{
        0.0 * x_extent[cm], extent_cm * x_extent[cm],
        0.0 * y_extent[cm], extent_cm * y_extent[cm],
        0.0 * z_extent[cm], extent_cm * z_extent[cm]};
}

// Matches the physical extent of single_voxel_x4_y4_z4.npy (5 voxels * 10cm).
MapConfig fileMapConfig() {
    return MapConfig{boundsFromExtentCm(50.0), Position3D{}, res10()};
}

// A fresh 10x10x10 voxel map (100cm per axis at 10cm resolution), entirely Unmapped.
MapConfig freshMapConfig() {
    return MapConfig{boundsFromExtentCm(100.0), Position3D{}, res10()};
}

std::shared_ptr<NpyArray> loadNpy(const std::filesystem::path& path) {
    auto array = std::make_shared<NpyArray>();
    array->LoadNPY(path.string());
    return array;
}

std::shared_ptr<NpyArray> emptyNpy() {
    return std::make_shared<NpyArray>();
}

constexpr double kEpsilon = 1e-9;

} // namespace

// ── config() roundtrip ───────────────────────────────────────────────────────

TEST(MockLidar, ConfigReturnsConstructorValue) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/2};
    MockLidar lidar{config, map, *gps};

    const LidarConfigData returned = lidar.config();

    EXPECT_EQ(returned.z_min, config.z_min);
    EXPECT_EQ(returned.z_max, config.z_max);
    EXPECT_EQ(returned.d, config.d);
    EXPECT_EQ(returned.fov_circles, config.fov_circles);
}

// ── hit / miss behavior ──────────────────────────────────────────────────────

TEST(MockLidar, ScanTowardOccupiedVoxelReturnsHitWithinSensorRange) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    // GPS sits below the occupied voxel and looks straight up (altitude=90deg),
    // so the beam marches in +z from (45,45,0).
    const auto gps = makeGps(pos(45.0, 45.0, 0.0), headingDeg(0.0, 90.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/200.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    // Center beam: scan_orientation is added to the sensor heading, so zero
    // offset means the beam points exactly along the heading (straight up).
    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_GE(result[0].distance, config.z_min);
    EXPECT_LE(result[0].distance, config.z_max);
    // The beam enters the occupied voxel's z-slab [40,50) at z=40, i.e. after
    // 40cm of travel from z=0.
    EXPECT_EQ(result[0].distance, 40.0 * cm);
}

TEST(MockLidar, ScanTowardEmptySpaceReturnsMiss) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    // GPS at the map's origin corner, looking straight down (altitude=-90deg):
    // the beam immediately leaves the map (min_height=0) and never finds an
    // Occupied voxel.
    const auto gps = makeGps(pos(0.0, 0.0, 0.0), headingDeg(0.0, -90.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].distance, std::numeric_limits<double>::max() * cm);
}

// ── fov_circles -> beam count ────────────────────────────────────────────────

TEST(MockLidar, FovCirclesOneReturnsSingleBeam) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    EXPECT_EQ(result.size(), 1u);
}

TEST(MockLidar, FovCirclesTwoReturnsFiveBeams) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    // 1 center beam + 4 beams on circle 1.
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/2};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    EXPECT_EQ(result.size(), 5u);
}

TEST(MockLidar, FovCirclesThreeReturnsTwentyOneBeams) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    // 1 center beam + 4 beams on circle 1 + 16 beams on circle 2.
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/3};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    EXPECT_EQ(result.size(), 21u);
}

TEST(MockLidar, FovCirclesZeroReturnsEmptyResult) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/0};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    EXPECT_TRUE(result.empty());
}

// ── z_min cutoff ──────────────────────────────────────────────────────────────

TEST(MockLidar, HitCloserThanZMinReturnsZero) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Occupied voxel directly above the GPS position, in the very first
    // z-slab [0,10) -- i.e. at distance 0cm from the GPS along the beam.
    map.set(pos(45.0, 45.0, 5.0), VoxelOccupancy::Occupied);

    // GPS sits inside that same voxel's column and looks straight up.
    const auto gps = makeGps(pos(45.0, 45.0, 0.0), headingDeg(0.0, 90.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    // The beam hits an Occupied voxel at distance 0cm, which is < z_min=10cm,
    // so traceBeam reports 0cm rather than the (sub-z_min) hit distance.
    EXPECT_EQ(result[0].distance, 0.0 * cm);
}

TEST(MockLidar, HitExactlyAtZMinReturnsActualDistanceNotZero) {
    // traceBeam's zero-clamp condition is `distance < config_.z_min` -- strictly less than. A hit
    // at exactly z_min must report that real distance, not be clamped to 0 like a sub-z_min hit
    // (HitCloserThanZMinReturnsZero above). A bug that changes the comparison to `<=` would fail
    // this without affecting the sub-z_min case at all.
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Occupied voxel at world x:[30,40); GPS at x=0 looking along +x, so the beam reaches x=30
    // (the front face of that voxel) at distance exactly 30cm.
    map.set(pos(35.0, 50.0, 50.0), VoxelOccupancy::Occupied);
    const auto gps = makeGps(pos(0.0, 50.0, 50.0), headingDeg(0.0, 0.0));

    const LidarConfigData config{/*z_min=*/30.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].distance, 30.0 * cm)
        << "a hit exactly at z_min must report the real distance, not be clamped to 0cm";
}

// ── z_max boundary ───────────────────────────────────────────────────────────

TEST(MockLidar, HitExactlyAtZMaxIsStillDetectedNotExcluded) {
    // traceBeam's loop condition is `distance <= config_.z_max` -- inclusive. A bug that changes
    // this to `<` would stop sampling one step before z_max and miss a hit placed exactly there,
    // silently reporting a miss (the sentinel "never returns" distance) instead.
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Occupied voxel at world z:[50,60); GPS at z=0 looking straight up, so the beam reaches z=50
    // (the bottom face of that voxel) at distance exactly 50cm == z_max.
    map.set(pos(45.0, 45.0, 55.0), VoxelOccupancy::Occupied);
    const auto gps = makeGps(pos(45.0, 45.0, 0.0), headingDeg(0.0, 90.0));

    const LidarConfigData config{/*z_min=*/5.0 * cm, /*z_max=*/50.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].distance, 50.0 * cm)
        << "z_max is an inclusive upper bound, so a hit placed exactly at that distance must "
           "still be detected, not treated as out of range";
}

// ── beam direction geometry ──────────────────────────────────────────────────

TEST(MockLidar, DiagonalBeamHitsVoxelAlongDiagonal) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Occupied voxel at world [30,40) x [30,40) x [50,60).
    map.set(pos(35.0, 35.0, 55.0), VoxelOccupancy::Occupied);

    // GPS at the origin column, looking along the horizontal 45-degree
    // diagonal (heading altitude=0 keeps z constant at 50cm).
    const auto gps = makeGps(pos(0.0, 0.0, 50.0), headingDeg(45.0, 0.0));

    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 1u);
    // At step distance d, x == y == d * cos(45deg). The beam enters the
    // occupied voxel once x == y >= 30cm, i.e. d >= 30 / cos(45deg) ~= 42.43cm.
    // With 1cm steps (0.1 * resolution), the first sample inside is d=43cm.
    EXPECT_EQ(result[0].distance, 43.0 * cm);
}

TEST(MockLidar, OffCenterBeamHitsWhileCenterBeamMisses) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Same target voxel as DiagonalBeamHitsVoxelAlongDiagonal.
    map.set(pos(35.0, 35.0, 55.0), VoxelOccupancy::Occupied);

    // GPS at the origin column, looking straight along +x (y stays 0).
    const auto gps = makeGps(pos(0.0, 0.0, 50.0), headingDeg(0.0, 0.0));

    // d == z_min => circle-1 beams are offset from the center beam by
    // atan2(d, z_min) = atan2(1,1) = 45 degrees (see horizontal_delta /
    // altitude_delta in MockLidar.cpp).
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/2};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 5u);

    // Center beam travels along +x at y=0 (index 0) and never reaches the
    // voxel at y-index 3: miss.
    EXPECT_EQ(result[0].distance, std::numeric_limits<double>::max() * cm);

    // Circle-1 beam 0 (theta=0 => horizontal_offset=d, altitude_offset=0) is
    // offset by +45 degrees horizontally, putting it on the same diagonal as
    // DiagonalBeamHitsVoxelAlongDiagonal -- it hits the same voxel at 43cm.
    EXPECT_EQ(result[1].distance, 43.0 * cm);
    EXPECT_NEAR(result[1].angle.horizontal.force_numerical_value_in(deg), 45.0, kEpsilon);
    EXPECT_NEAR(result[1].angle.altitude.force_numerical_value_in(deg), 0.0, kEpsilon);
}

TEST(MockLidar, CircleBeamsAtDifferentIndicesHaveDistinctAngles) {
    // theta = 360 * i / beam_count must use floating-point division -- a bug that performs the
    // division in integer arithmetic first (e.g. `360.0 * static_cast<double>(i / beam_count)`)
    // truncates i/beam_count to 0 for every i < beam_count, silently collapsing every beam on the
    // circle onto the same angle as beam index 0. OffCenterBeamHitsWhileCenterBeamMisses above
    // only inspects beam index 0, where 0/beam_count==0 regardless of truncation, so it cannot
    // catch this; this test pins down a beam index where the two computations diverge.
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(0.0, 0.0, 50.0), headingDeg(0.0, 0.0));

    // d == z_min, same geometry as OffCenterBeamHitsWhileCenterBeamMisses: circle-1 beams are
    // offset from the center beam by atan2(d, z_min) = 45 degrees, but on different axes
    // depending on theta.
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/2};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 5u);

    // Beam index 0 (theta=0deg): horizontal_offset=d, altitude_offset=0 -> +45deg horizontal, 0deg
    // altitude (verified already above; repeated here only as the baseline for comparison).
    EXPECT_NEAR(result[1].angle.horizontal.force_numerical_value_in(deg), 45.0, kEpsilon);
    EXPECT_NEAR(result[1].angle.altitude.force_numerical_value_in(deg), 0.0, kEpsilon);

    // Beam index 1 (theta=90deg): horizontal_offset=0, altitude_offset=d -> 0deg horizontal,
    // +45deg altitude. Under the integer-division bug this would wrongly equal beam index 0's
    // angle (45deg horizontal, 0deg altitude) instead.
    EXPECT_NEAR(result[2].angle.horizontal.force_numerical_value_in(deg), 0.0, kEpsilon)
        << "beam index 1's angle must reflect theta=90deg, not collapse to beam index 0's angle";
    EXPECT_NEAR(result[2].angle.altitude.force_numerical_value_in(deg), 45.0, kEpsilon)
        << "beam index 1's angle must reflect theta=90deg, not collapse to beam index 0's angle";

    // Beam index 2 (theta=180deg): horizontal_offset=-d, altitude_offset=0 -> -45deg horizontal,
    // 0deg altitude.
    EXPECT_NEAR(result[3].angle.horizontal.force_numerical_value_in(deg), -45.0, kEpsilon)
        << "beam index 2's angle must reflect theta=180deg, not collapse to beam index 0's angle";
    EXPECT_NEAR(result[3].angle.altitude.force_numerical_value_in(deg), 0.0, kEpsilon);
}

TEST(MockLidar, OuterCircleBeamRadiusScalesWithCircleIndex) {
    // The per-circle beam-offset radius must be `circle_index * config_.d`, not a fixed
    // `config_.d` for every non-center circle. A bug that drops the `circle *` scaling factor
    // would make circle 2's beams use the same radius as circle 1's, which is invisible to
    // OffCenterBeamHitsWhileCenterBeamMisses/CircleBeamsAtDifferentIndicesHaveDistinctAngles
    // above (both only exercise circle 1, where `1 * d == d` either way). This test checks
    // circle 2's beam-0 angle, which only matches `radius = 2 * d` -- not `radius = d`.
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto gps = makeGps(pos(0.0, 0.0, 50.0), headingDeg(0.0, 0.0));

    // d == z_min, fov_circles=3 -> 1 center + 4 (circle 1) + 16 (circle 2) beams.
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/3};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult result = lidar.scan(headingDeg(0.0, 0.0));

    ASSERT_EQ(result.size(), 21u);

    // Circle 2, beam index 0 (theta=0deg): horizontal_offset = radius = 2*d, altitude_offset=0.
    // horizontal angle = atan2(2*d, z_min) = atan2(2,1) ~= 63.43494882292201 degrees with the
    // correct radius=2*d; a bug that uses radius=d (no circle scaling) would instead produce 45
    // degrees, identical to circle 1's beam 0.
    constexpr double expected_horizontal_deg = 63.43494882292201;
    ASSERT_GT(result.size(), 5u);
    EXPECT_NEAR(result[5].angle.horizontal.force_numerical_value_in(deg), expected_horizontal_deg,
                kEpsilon)
        << "circle 2's beam-0 offset radius must be 2*d (scaled by circle index), not a fixed d "
           "shared with circle 1";
    EXPECT_NEAR(result[5].angle.altitude.force_numerical_value_in(deg), 0.0, kEpsilon);
}

// ── scan_orientation argument actually steers the beam (LID03 coverage gap) ──

// New — closes a Project 2 mutation-coverage gap (LID03, see PROJ2_TESTS_PLAN.md
// §8.1/§9). Every test above holds the scan_orientation *argument* fixed at
// (0,0) and instead varies the beam direction via GPS position/heading --
// none of them would notice a bug that silently discards the orientation
// argument passed into scan() (LID03's mutation does exactly that:
// `scan_orientation = Orientation{};` at the top of scan(), before it's
// combined with gps_.heading()). This test isolates the orientation
// *parameter* specifically: GPS heading is held at zero, so each call's
// result can only differ because of the orientation argument itself.
TEST(MockLidar, ScanOrientationArgumentDeterminesBeamDirection) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    // Obstacle only along +x from the GPS position -- +y is entirely empty.
    map.set(pos(80.0, 50.0, 50.0), VoxelOccupancy::Occupied);

    const auto gps = makeGps(pos(50.0, 50.0, 50.0), headingDeg(0.0, 0.0));
    const LidarConfigData config{/*z_min=*/10.0 * cm, /*z_max=*/100.0 * cm, /*d=*/10.0 * cm, /*fov_circles=*/1};
    MockLidar lidar{config, map, *gps};

    const LidarScanResult toward_obstacle = lidar.scan(headingDeg(0.0, 0.0));     // +x: hits
    const LidarScanResult away_from_obstacle = lidar.scan(headingDeg(90.0, 0.0)); // +y: misses

    ASSERT_EQ(toward_obstacle.size(), 1u);
    ASSERT_EQ(away_from_obstacle.size(), 1u);
    EXPECT_NE(toward_obstacle[0].distance, std::numeric_limits<double>::max() * cm)
        << "beam aimed at the obstacle via the orientation argument should hit it";
    EXPECT_EQ(away_from_obstacle[0].distance, std::numeric_limits<double>::max() * cm)
        << "beam aimed away from the obstacle via the orientation argument should miss";
}
