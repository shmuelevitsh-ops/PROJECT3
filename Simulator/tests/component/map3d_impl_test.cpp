// Migrated from Project 2 (FILES PROJECT 2/tests/components/map3d_impl_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespaces/includes only, no other behavioral changes from the Project 2 original.

#include <Simulator/Map3DImpl.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <typeinfo>

using namespace common;
using namespace common::types;
using namespace simulator;

#ifndef DATA_MAPS_DIR
#define DATA_MAPS_DIR "."
#endif

namespace {

const std::filesystem::path k_npy =
    std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy"; // 5x5x5, voxel (4,4,4) occupied

PhysicalLength res10() {
    return 10.0 * isq::length[cm];
}

Position3D pos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
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

// fileMapConfig() boundaries narrowed to [0,30) on every axis (3 voxels) —
// strictly smaller than single_voxel_x4_y4_z4.npy's 50cm physical extent,
// so construction does not clamp them (clamping only shrinks boundaries
// that exceed the array's extent, never boundaries already smaller than it).
// The array's physical extent is therefore intentionally larger than the
// configured boundaries.
MapConfig narrowBoundsConfig() {
    MapConfig config = fileMapConfig();
    config.boundaries.max_x = 30.0 * x_extent[cm];
    config.boundaries.max_y = 30.0 * y_extent[cm];
    config.boundaries.max_height = 30.0 * z_extent[cm];
    return config;
}

// A fresh 10x10x10 voxel map (100cm per axis at 10cm resolution).
MapConfig freshMapConfig() {
    return MapConfig{boundsFromExtentCm(100.0), Position3D{}, res10()};
}

std::shared_ptr<NpyArray> loadNpy(const std::filesystem::path& path) {
    auto array = std::make_shared<NpyArray>();
    const char* error = array->LoadNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error("Failed to load test NPY file '" + path.string() + "': " + error);
    }
    return array;
}

std::shared_ptr<NpyArray> emptyNpy() {
    return std::make_shared<NpyArray>();
}

// Builds an owned int32 NpyArray with the given shape, filled with `fill`.
std::shared_ptr<NpyArray> makeIntArray(NpyArray::shape_t shape, int fill) {
    auto array =
        std::make_shared<NpyArray>(std::move(shape), sizeof(int), NpyArray::GetTypeChar(typeid(int)));
    array->Allocate();
    std::fill_n(array->Data<int>(), array->NumValue(), fill);
    return array;
}

} // namespace

// ── file-loaded map ──────────────────────────────────────────────────────────

TEST(Map3DImpl, FileConstructorDoesNotThrow) {
    EXPECT_NO_THROW((Map3DImpl{loadNpy(k_npy), fileMapConfig()}));
}

TEST(Map3DImpl, FileConstructorOccupiedVoxelAt40cm) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(40.0, 40.0, 40.0)), VoxelOccupancy::Occupied);
}

TEST(Map3DImpl, FileConstructorEmptyVoxelAt0cm) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::Empty);
}

TEST(Map3DImpl, FileConstructorOutOfBounds) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(9999.0, 9999.0, 9999.0)), VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, NonThreeDimensionalArrayThrowsInvalidArgument) {
    auto array = makeIntArray(NpyArray::shape_t{5, 5}, 0);
    EXPECT_THROW((Map3DImpl{array, fileMapConfig()}), std::invalid_argument);
}

// ── bounds-constructed (fresh) map ───────────────────────────────────────────

TEST(Map3DImpl, BoundsConstructorFreshMapIsUnmapped) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(50.0, 50.0, 50.0)), VoxelOccupancy::Unmapped);
}

TEST(Map3DImpl, BoundsConstructorSetOccupiedRoundtrip) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto p = pos(50.0, 50.0, 50.0);
    map.set(p, VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(p), VoxelOccupancy::Occupied);
}

TEST(Map3DImpl, BoundsConstructorSetEmptyRoundtrip) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto p = pos(50.0, 50.0, 50.0);
    map.set(p, VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(p), VoxelOccupancy::Empty);
}

TEST(Map3DImpl, BoundsConstructorSetOutOfBoundsDoesNotThrow) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    EXPECT_NO_THROW(map.set(pos(9999.0, 9999.0, 9999.0), VoxelOccupancy::Occupied));
}

TEST(Map3DImpl, BoundsConstructorOutOfBoundsIsNotUnmapped) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(9999.0, 9999.0, 9999.0)), VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, SetOutOfBoundsValueOnInBoundsPositionDoesNotOverwrite) {
    // OutOfBounds is a read-only sentinel, not a value a caller should be
    // able to store; set() must ignore it without throwing or touching the
    // existing stored value.
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const auto p = pos(50.0, 50.0, 50.0);
    map.set(p, VoxelOccupancy::Occupied);

    EXPECT_NO_THROW(map.set(p, VoxelOccupancy::OutOfBounds));
    EXPECT_EQ(map.atVoxel(p), VoxelOccupancy::Occupied);
}

// ── access via IMap3D (e.g. MappingAlgorithmImpl's output_map) ──────────────

TEST(Map3DImpl, FreshMapBehavesCorrectlyThroughIMap3DInterface) {
    Map3DImpl map{emptyNpy(), freshMapConfig()};
    const IMap3D& output_map = map;

    EXPECT_EQ(output_map.getMapConfig().resolution, freshMapConfig().resolution);

    const auto p = pos(50.0, 50.0, 50.0);
    EXPECT_EQ(output_map.atVoxel(p), VoxelOccupancy::Unmapped);

    map.set(p, VoxelOccupancy::Occupied);
    EXPECT_EQ(output_map.atVoxel(p), VoxelOccupancy::Occupied);

    EXPECT_EQ(output_map.atVoxel(pos(9999.0, 9999.0, 9999.0)), VoxelOccupancy::OutOfBounds);
}

// ── save / reload ─────────────────────────────────────────────────────────────

TEST(Map3DImpl, SaveThenReloadPreservesOccupiedVoxel) {
    const auto tmp = std::filesystem::temp_directory_path() / "map3d_impl_test.npy";
    const auto p = pos(50.0, 50.0, 50.0);
    {
        Map3DImpl map{emptyNpy(), freshMapConfig()};
        map.set(p, VoxelOccupancy::Occupied);
        map.save(tmp);
    }
    Map3DImpl reloaded{loadNpy(tmp), freshMapConfig()};
    EXPECT_EQ(reloaded.atVoxel(p), VoxelOccupancy::Occupied);
    std::filesystem::remove(tmp);
}

// ── stored raw value mapping ─────────────────────────────────────────────────

TEST(Map3DImpl, StoredValueZeroIsEmpty) {
    Map3DImpl map{makeIntArray({1, 1, 1}, 0), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::Empty);
}

TEST(Map3DImpl, StoredValueOneIsOccupied) {
    Map3DImpl map{makeIntArray({1, 1, 1}, 1), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::Occupied);
}

TEST(Map3DImpl, StoredValueMinusOneIsUnmapped) {
    Map3DImpl map{makeIntArray({1, 1, 1}, -1), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::Unmapped);
}

TEST(Map3DImpl, StoredValueMinusTwoIsTreatedAsUnmapped) {
    // -2 should never be stored, but if it is, it must be logged and treated
    // as Unmapped (not silently ignored, and not OutOfBounds).
    Map3DImpl map{makeIntArray({1, 1, 1}, -2), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::Unmapped);
}

TEST(Map3DImpl, OutOfRangeIndexIsOutOfBoundsNotUnmapped) {
    Map3DImpl map{makeIntArray({1, 1, 1}, -1), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(100.0, 100.0, 100.0)), VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, StoredValueMinusThreeIsPotentiallyOccupied) {
    Map3DImpl map{makeIntArray({1, 1, 1}, -3), fileMapConfig()};
    EXPECT_EQ(map.atVoxel(pos(0.0, 0.0, 0.0)), VoxelOccupancy::PotentiallyOccupied);
}

TEST(Map3DImpl, SetNegativeValueOnUint8MapDoesNotThrowAndIsIgnored) {
    // single_voxel_x4_y4_z4.npy is stored as uint8 (dtype '|u1'), which
    // cannot represent negative raw values (Unmapped=-1, PotentiallyOccupied=-3).
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    const auto p = pos(0.0, 0.0, 0.0);  // currently Empty (raw 0)

    EXPECT_NO_THROW(map.set(p, VoxelOccupancy::Unmapped));
    EXPECT_EQ(map.atVoxel(p), VoxelOccupancy::Empty)
        << "Negative raw values cannot be stored in a uint8 map; set() must be a no-op.";
}

// ── getMapConfig / boundary clamping ─────────────────────────────────────────

TEST(Map3DImpl, GetMapConfigReturnsResolutionFromConstruction) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_EQ(map.getMapConfig().resolution, res10());
}

TEST(Map3DImpl, GetMapConfigPreservesOffsetFromConstruction) {
    // Non-zero offset; boundaries span exactly 9 voxels at 10cm resolution
    // starting at the offset, so the array is allocated to fit them exactly
    // and no clamping occurs. getMapConfig() must return the same offset,
    // resolution, and boundaries that were passed at construction.
    const MapConfig config{
        MappingBounds{
            100.0 * x_extent[cm], 190.0 * x_extent[cm],
            200.0 * y_extent[cm], 290.0 * y_extent[cm],
            300.0 * z_extent[cm], 390.0 * z_extent[cm]},
        Position3D{100.0 * x_extent[cm], 200.0 * y_extent[cm], 300.0 * z_extent[cm]},
        res10()};

    Map3DImpl map{emptyNpy(), config};
    const MapConfig result = map.getMapConfig();

    EXPECT_EQ(result.offset.x, config.offset.x);
    EXPECT_EQ(result.offset.y, config.offset.y);
    EXPECT_EQ(result.offset.z, config.offset.z);
    EXPECT_EQ(result.resolution, config.resolution);
    EXPECT_EQ(result.boundaries.min_x, config.boundaries.min_x);
    EXPECT_EQ(result.boundaries.max_x, config.boundaries.max_x);
    EXPECT_EQ(result.boundaries.min_y, config.boundaries.min_y);
    EXPECT_EQ(result.boundaries.max_y, config.boundaries.max_y);
    EXPECT_EQ(result.boundaries.min_height, config.boundaries.min_height);
    EXPECT_EQ(result.boundaries.max_height, config.boundaries.max_height);
}

TEST(Map3DImpl, BoundariesWithinMapExtentAreUnchanged) {
    // fileMapConfig() boundaries exactly match the physical extent of k_npy
    // (50cm per axis), so construction must leave them unchanged.
    const MapConfig config = fileMapConfig();
    Map3DImpl map{loadNpy(k_npy), config};
    const MapConfig result = map.getMapConfig();
    EXPECT_EQ(result.boundaries.min_x, config.boundaries.min_x);
    EXPECT_EQ(result.boundaries.max_x, config.boundaries.max_x);
    EXPECT_EQ(result.boundaries.min_y, config.boundaries.min_y);
    EXPECT_EQ(result.boundaries.max_y, config.boundaries.max_y);
    EXPECT_EQ(result.boundaries.min_height, config.boundaries.min_height);
    EXPECT_EQ(result.boundaries.max_height, config.boundaries.max_height);
}

TEST(Map3DImpl, BoundariesStrictlyInsideMapExtentAreUnchanged) {
    // single_voxel_x4_y4_z4.npy has a 50cm physical extent on every axis;
    // [10, 40] is strictly inside that on every axis.
    MapConfig config = fileMapConfig();
    config.boundaries.min_x = 10.0 * x_extent[cm];
    config.boundaries.max_x = 40.0 * x_extent[cm];
    config.boundaries.min_y = 10.0 * y_extent[cm];
    config.boundaries.max_y = 40.0 * y_extent[cm];
    config.boundaries.min_height = 10.0 * z_extent[cm];
    config.boundaries.max_height = 40.0 * z_extent[cm];

    Map3DImpl map{loadNpy(k_npy), config};
    const MapConfig result = map.getMapConfig();
    EXPECT_EQ(result.boundaries.min_x, config.boundaries.min_x);
    EXPECT_EQ(result.boundaries.max_x, config.boundaries.max_x);
    EXPECT_EQ(result.boundaries.min_y, config.boundaries.min_y);
    EXPECT_EQ(result.boundaries.max_y, config.boundaries.max_y);
    EXPECT_EQ(result.boundaries.min_height, config.boundaries.min_height);
    EXPECT_EQ(result.boundaries.max_height, config.boundaries.max_height);
}

TEST(Map3DImpl, BoundariesExceedingMapExtentAreClamped) {
    MapConfig config = fileMapConfig();
    // single_voxel_x4_y4_z4.npy is 5 voxels at 10cm => 50cm physical extent.
    config.boundaries.max_x = 1000.0 * x_extent[cm];
    config.boundaries.max_y = 1000.0 * y_extent[cm];
    config.boundaries.max_height = 1000.0 * z_extent[cm];

    Map3DImpl map{loadNpy(k_npy), config};
    const MapConfig result = map.getMapConfig();
    EXPECT_EQ(result.boundaries.max_x, 50.0 * x_extent[cm]);
    EXPECT_EQ(result.boundaries.max_y, 50.0 * y_extent[cm]);
    EXPECT_EQ(result.boundaries.max_height, 50.0 * z_extent[cm]);
}

// Map3DImpl reports boundary clamping via std::cerr. This test captures and
// restores std::cerr locally, independent of the simulator's error-log
// redirect.
TEST(Map3DImpl, BoundariesExceedingMapExtentAreLoggedToCerr) {
    MapConfig config = fileMapConfig();
    // single_voxel_x4_y4_z4.npy is 5 voxels at 10cm => 50cm physical extent.
    config.boundaries.max_x = 1000.0 * x_extent[cm];

    std::ostringstream captured_cerr;
    std::streambuf* original_cerr = std::cerr.rdbuf(captured_cerr.rdbuf());
    Map3DImpl map{loadNpy(k_npy), config};
    std::cerr.rdbuf(original_cerr);

    EXPECT_FALSE(captured_cerr.str().empty())
        << "Expected Map3DImpl to log a clamping warning to std::cerr when "
           "boundaries exceed the map extent.";
}

TEST(Map3DImpl, ClampingAfterDoesNotAffectAtVoxelBehavior) {
    MapConfig config = fileMapConfig();
    config.boundaries.max_x = 1000.0 * x_extent[cm];

    Map3DImpl map{loadNpy(k_npy), config};
    EXPECT_EQ(map.atVoxel(pos(40.0, 40.0, 40.0)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(pos(9999.0, 9999.0, 9999.0)), VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, DisjointBoundariesThrowInvalidArgument) {
    // single_voxel_x4_y4_z4.npy is 5 voxels at 10cm => 50cm physical extent.
    MapConfig config = fileMapConfig();
    config.boundaries.min_x = 1000.0 * x_extent[cm];
    config.boundaries.max_x = 2000.0 * x_extent[cm];

    EXPECT_THROW((Map3DImpl{loadNpy(k_npy), config}), std::invalid_argument);
}

TEST(Map3DImpl, SingleArgumentConstructorInfersDefaultConfigFromShape) {
    Map3DImpl map{loadNpy(k_npy)};
    const MapConfig config = map.getMapConfig();

    EXPECT_EQ(config.resolution, 1.0 * isq::length[cm]);
    EXPECT_EQ(config.offset.x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.offset.y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.offset.z, 0.0 * z_extent[cm]);
    // single_voxel_x4_y4_z4.npy has shape [5,5,5]; at 1cm resolution the
    // inferred boundaries must match the shape exactly.
    EXPECT_EQ(config.boundaries.min_x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 5.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.min_y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.max_y, 5.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.min_height, 0.0 * z_extent[cm]);
    EXPECT_EQ(config.boundaries.max_height, 5.0 * z_extent[cm]);
}

TEST(Map3DImpl, SingleArgumentConstructorThrowsForEmptyNpyArray) {
    EXPECT_THROW((Map3DImpl{emptyNpy()}), std::invalid_argument);
}

TEST(Map3DImpl, ExplicitConfigWithNonPositiveResolutionThrows) {
    MapConfig config = fileMapConfig();
    config.resolution = 0.0 * isq::length[cm];

    EXPECT_THROW((Map3DImpl{loadNpy(k_npy), config}), std::invalid_argument);
}

TEST(Map3DImpl, InvalidBoundariesMinGreaterOrEqualMaxThrowsBeforeClamping) {
    // min_x == max_x is invalid regardless of clamping (the array's physical
    // extent is 50cm, so this would also be valid-but-clamped if it weren't
    // already rejected before clamping is attempted).
    MapConfig config = fileMapConfig();
    config.boundaries.min_x = 40.0 * x_extent[cm];
    config.boundaries.max_x = 40.0 * x_extent[cm];

    try {
        Map3DImpl map{loadNpy(k_npy), config};
        FAIL() << "Expected std::invalid_argument to be thrown.";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("invalid mapping boundaries"), std::string::npos)
            << "Expected a pre-clamp invalid-boundaries message, got: " << e.what();
    }
}

// ── fresh-map allocation: configured offset vs. boundary minimum ────────────

TEST(Map3DImpl, FreshMapConstructionThrowsWhenBoundaryMinMapsToNegativeIndex) {
    // offset=0, resolution=10cm; min_x=-10cm => index floor(-10/10) = -1.
    MapConfig config = freshMapConfig();
    config.boundaries.min_x = -10.0 * x_extent[cm];

    EXPECT_THROW((Map3DImpl{emptyNpy(), config}), std::invalid_argument);
}

TEST(Map3DImpl, FreshMapConstructionSucceedsWhenBoundaryMinMapsToNonNegativeIndex) {
    // offset=0, resolution=10cm; min_x=20cm => index floor(20/10) = 2 (>= 0).
    MapConfig config = freshMapConfig();
    config.boundaries.min_x = 20.0 * x_extent[cm];

    EXPECT_NO_THROW((Map3DImpl{emptyNpy(), config}));
}

TEST(Map3DImpl, NegativeOffsetWithBoundariesAtOffsetIsValid) {
    const MapConfig config{
        MappingBounds{
            -50.0 * x_extent[cm], 50.0 * x_extent[cm],
            -50.0 * y_extent[cm], 50.0 * y_extent[cm],
            -50.0 * z_extent[cm], 50.0 * z_extent[cm]},
        Position3D{-50.0 * x_extent[cm], -50.0 * y_extent[cm], -50.0 * z_extent[cm]},
        res10()};

    Map3DImpl map{emptyNpy(), config};
    map.set(pos(-50.0, -50.0, -50.0), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(pos(-50.0, -50.0, -50.0)), VoxelOccupancy::Occupied);
    EXPECT_TRUE(map.isInBounds(pos(0.0, 0.0, 0.0)));
}

TEST(Map3DImpl, FreshMapAllocationSizedFromOffsetNotBoundaryMinimum) {
    // offset=0, boundaries=[20,100) cm on every axis, resolution=10cm.
    // offset < boundaries.min: the array must be sized to cover [offset, max)
    // = [0,100), not just the [20,100) span, so the configured boundaries are
    // not clamped away after construction.
    MapConfig config = freshMapConfig();
    config.boundaries.min_x = 20.0 * x_extent[cm];
    config.boundaries.min_y = 20.0 * y_extent[cm];
    config.boundaries.min_height = 20.0 * z_extent[cm];

    Map3DImpl map{emptyNpy(), config};

    const MapConfig result = map.getMapConfig();
    EXPECT_EQ(result.boundaries.min_x, config.boundaries.min_x);
    EXPECT_EQ(result.boundaries.max_x, config.boundaries.max_x);
    EXPECT_EQ(result.boundaries.min_y, config.boundaries.min_y);
    EXPECT_EQ(result.boundaries.max_y, config.boundaries.max_y);
    EXPECT_EQ(result.boundaries.min_height, config.boundaries.min_height);
    EXPECT_EQ(result.boundaries.max_height, config.boundaries.max_height);

    // A position near the upper boundary (max=100, exclusive) must be
    // writable/readable without having been clamped away.
    const auto p = pos(90.0, 90.0, 90.0);
    map.set(p, VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(p), VoxelOccupancy::Occupied);
}

// ── isInBounds ───────────────────────────────────────────────────────────────

TEST(Map3DImpl, IsInBoundsReturnsTrueForValidPosition) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_TRUE(map.isInBounds(pos(0.0, 0.0, 0.0)));
}

TEST(Map3DImpl, IsInBoundsReturnsFalseForOutOfBoundsPosition) {
    Map3DImpl map{loadNpy(k_npy), fileMapConfig()};
    EXPECT_FALSE(map.isInBounds(pos(9999.0, 9999.0, 9999.0)));
}

// ── configured boundaries narrower than the NpyArray's physical extent ──────
// The array extent being larger than the configured boundaries is a valid
// configuration (not an error): isInBounds()/atVoxel()/set() must honor the
// configured boundaries, not merely whether the array has a voxel there.

TEST(Map3DImpl, IsInBoundsReturnsTrueInsideConfiguredBoundaries) {
    Map3DImpl map{loadNpy(k_npy), narrowBoundsConfig()};
    EXPECT_TRUE(map.isInBounds(pos(10.0, 10.0, 10.0)));
}

TEST(Map3DImpl, IsInBoundsReturnsFalseInsideArrayButOutsideConfiguredBoundaries) {
    Map3DImpl map{loadNpy(k_npy), narrowBoundsConfig()};
    // (40,40,40) maps to a valid index inside the 5x5x5 array (the occupied
    // voxel), but is outside the configured [0,30) boundaries.
    EXPECT_FALSE(map.isInBounds(pos(40.0, 40.0, 40.0)));
}

TEST(Map3DImpl, AtVoxelReturnsOutOfBoundsInsideArrayButOutsideConfiguredBoundaries) {
    Map3DImpl map{loadNpy(k_npy), narrowBoundsConfig()};
    EXPECT_EQ(map.atVoxel(pos(40.0, 40.0, 40.0)), VoxelOccupancy::OutOfBounds)
        << "(40,40,40) is a valid (and Occupied) array index, but outside the "
           "configured boundaries; atVoxel() must report OutOfBounds, not "
           "the raw stored value.";
}

TEST(Map3DImpl, SetDoesNotWriteInsideArrayButOutsideConfiguredBoundaries) {
    auto array = loadNpy(k_npy);
    Map3DImpl map{array, narrowBoundsConfig()};
    const auto p = pos(40.0, 40.0, 40.0);

    map.set(p, VoxelOccupancy::Empty);

    // Read back through a second Map3DImpl sharing the same underlying array
    // but with the full (unnarrowed) boundaries, to confirm set() never
    // touched the array.
    Map3DImpl unmasked{array, fileMapConfig()};
    EXPECT_EQ(unmasked.atVoxel(p), VoxelOccupancy::Occupied)
        << "set() must not write outside the configured boundaries, even "
           "though the position maps to a valid NpyArray index.";
}
