// Migrated from Project 2 (FILES PROJECT 2/tests/components/maps_comparison_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespaces/includes only for the MapsComparison::compare() tests below, no other
// behavioral changes from the Project 2 original.
//
// Deviation from the migration manifest's per-file count (39 -> 27 tests here): the
// Project 2 source file also contained 12 tests exercising `drone_mapper::run()`, the
// maps_comparison CLI entry point's in-process run() helper. Project 3 has no
// `maps_comparison` CLI executable or *_main.cpp equivalent anywhere in the tree (the
// same fact already used in the manifest to mark the separate
// `maps_comparison_cli_test.cpp` file as "not migrated (deferred)") -- those 12 tests
// have nothing to compile or run against here and are excluded for the same reason,
// not migrated as part of this file. Revisit both together if/when such a target is
// added.

#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>
#include <TinyNPY.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>

// Note: deliberately not `using namespace simulator;` -- the test fixture below is
// named `MapsComparison` (required by the assignment's gtest filter) which would
// collide with `simulator::MapsComparison`. Calls to the class under test use the
// fully-qualified name `simulator::MapsComparison::compare`.
using namespace common;
using namespace common::types;

using simulator::Map3DImpl;

namespace {

constexpr double kResolutionCm = 10.0;

PhysicalLength resolution10() {
    return kResolutionCm * isq::length[cm];
}

Position3D pos(double x_cm, double y_cm, double z_cm) {
    return Position3D{x_cm * x_extent[cm], y_cm * y_extent[cm], z_cm * z_extent[cm]};
}

MappingBounds bounds(double min_x, double max_x, double min_y, double max_y, double min_z,
                      double max_z) {
    return MappingBounds{
        min_x * x_extent[cm], max_x * x_extent[cm],
        min_y * y_extent[cm], max_y * y_extent[cm],
        min_z * z_extent[cm], max_z * z_extent[cm]};
}

// A MapConfig whose offset equals `b`'s minimum corner (so world position ==
// offset + index * resolution, with no boundary clamping).
MapConfig configFor(const MappingBounds& b, PhysicalLength res = resolution10()) {
    return MapConfig{b, Position3D{b.min_x, b.min_y, b.min_height}, res};
}

std::shared_ptr<NpyArray> emptyNpy() {
    return std::make_shared<NpyArray>();
}

// A minimal IMap3D stub that returns a fixed VoxelOccupancy value (or
// Unmapped, via the single-argument constructor) from every atVoxel() call,
// regardless of position. Used both to exercise compare()'s validation checks
// against MapConfig values that Map3DImpl's constructor would reject outright
// (e.g. degenerate boundaries trigger Map3DImpl's disjoint-boundary throw),
// and to isolate scoring tests that only need uniform map content from any
// possible Map3DImpl bug.
class FakeMap3D : public IMap3D {
public:
    explicit FakeMap3D(MapConfig config) : config_(config) {}
    FakeMap3D(MapConfig config, VoxelOccupancy value) : config_(config), value_(value) {}

    [[nodiscard]] VoxelOccupancy atVoxel(const Position3D& /*pos*/) const override {
        return value_;
    }
    [[nodiscard]] MapConfig getMapConfig() const override { return config_; }
    [[nodiscard]] bool isInBounds(const Position3D& /*pos*/) const override { return true; }

private:
    MapConfig config_;
    VoxelOccupancy value_ = VoxelOccupancy::Unmapped;
};

// Like FakeMap3D, but the per-voxel value is computed by a caller-supplied rule instead of being
// fixed. Lets tests that need content to vary by world position (e.g. "Occupied left of x=35cm,
// Empty elsewhere") avoid depending on Map3DImpl's own (separately tested) set()/atVoxel() storage
// and indexing behavior to set that content up.
class RuleMap3D : public IMap3D {
public:
    RuleMap3D(MapConfig config, std::function<VoxelOccupancy(const Position3D&)> rule)
        : config_(config), rule_(std::move(rule)) {}

    [[nodiscard]] VoxelOccupancy atVoxel(const Position3D& world_pos) const override {
        return rule_(world_pos);
    }
    [[nodiscard]] MapConfig getMapConfig() const override { return config_; }
    [[nodiscard]] bool isInBounds(const Position3D& /*pos*/) const override { return true; }

private:
    MapConfig config_;
    std::function<VoxelOccupancy(const Position3D&)> rule_;
};

// True if world position `p` falls inside `b` on every axis (half-open, [min, max)).
bool withinBounds(const Position3D& p, const MappingBounds& b) {
    return p.x >= b.min_x && p.x < b.max_x && p.y >= b.min_y && p.y < b.max_y && p.z >= b.min_height &&
          p.z < b.max_height;
}

} // namespace

class MapsComparison : public ::testing::Test {
protected:
    // Builds a fresh bounds-constructed map whose offset equals the
    // boundaries' minimum corner (so world position == offset + index *
    // resolution, with no boundary clamping), filled uniformly with `fill`.
    static std::shared_ptr<Map3DImpl> makeUniformMap(const MappingBounds& b, VoxelOccupancy fill,
                                                      PhysicalLength res = resolution10()) {
        auto map = std::make_shared<Map3DImpl>(emptyNpy(), configFor(b, res));
        if (fill != VoxelOccupancy::Unmapped) {
            fillBounds(*map, b, res, fill);
        }
        return map;
    }

    // Calls set(pos, value) for every voxel position covering `b` at step `res`.
    static void fillBounds(Map3DImpl& map, const MappingBounds& b, PhysicalLength res,
                            VoxelOccupancy value) {
        const double step = res.force_numerical_value_in(cm);
        const double min_x = b.min_x.force_numerical_value_in(cm);
        const double min_y = b.min_y.force_numerical_value_in(cm);
        const double min_z = b.min_height.force_numerical_value_in(cm);
        const auto nx = static_cast<long>(
            std::lround((b.max_x - b.min_x).force_numerical_value_in(cm) / step));
        const auto ny = static_cast<long>(
            std::lround((b.max_y - b.min_y).force_numerical_value_in(cm) / step));
        const auto nz = static_cast<long>(
            std::lround((b.max_height - b.min_height).force_numerical_value_in(cm) / step));

        for (long ix = 0; ix < nx; ++ix) {
            for (long iy = 0; iy < ny; ++iy) {
                for (long iz = 0; iz < nz; ++iz) {
                    map.set(pos(min_x + static_cast<double>(ix) * step,
                                 min_y + static_cast<double>(iy) * step,
                                 min_z + static_cast<double>(iz) * step),
                            value);
                }
            }
        }
    }
};

// ── validation / error tests ─────────────────────────────────────────────────

TEST_F(MapsComparison, ZeroResolutionOriginReturnsErrorForThatTarget) {
    const MappingBounds b = bounds(0, 50, 0, 50, 0, 50);
    FakeMap3D origin{configFor(b, 0.0 * isq::length[cm])};
    FakeMap3D target{configFor(b), VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 for a target when origin has non-positive resolution";
}

TEST_F(MapsComparison, ZeroResolutionTargetReturnsErrorForThatTarget) {
    const MappingBounds b = bounds(0, 50, 0, 50, 0, 50);
    FakeMap3D origin{configFor(b), VoxelOccupancy::Empty};
    FakeMap3D target{configFor(b, 0.0 * isq::length[cm])};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 for a target whose resolution is non-positive";
}

TEST_F(MapsComparison, NegativeResolutionOriginReturnsErrorForThatTarget) {
    // The zero-resolution tests above exercise validateComparable()'s `resolution > 0.0` check at
    // exactly 0; a negative resolution is a distinct boundary a mutated `resolution != 0.0` check
    // would wrongly accept.
    const MappingBounds b = bounds(0, 50, 0, 50, 0, 50);
    FakeMap3D origin{configFor(b, -10.0 * isq::length[cm])};
    FakeMap3D target{configFor(b), VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 for a target when origin has a negative resolution";
}

TEST_F(MapsComparison, MismatchedResolutionsReturnsErrorForThatTargetOnly) {
    FakeMap3D origin{configFor(bounds(0, 50, 0, 50, 0, 50), resolution10()),
                      VoxelOccupancy::Empty};
    FakeMap3D bad_target{configFor(bounds(0, 100, 0, 100, 0, 100), 20.0 * isq::length[cm]),
                          VoxelOccupancy::Empty};
    FakeMap3D good_target{configFor(bounds(0, 50, 0, 50, 0, 50), resolution10()),
                           VoxelOccupancy::Empty};

    const auto scores =
        simulator::MapsComparison::compare(origin, {&bad_target, &good_target});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 when origin and a target have different resolutions";
    EXPECT_DOUBLE_EQ(scores[1], 100.0)
        << "A resolution mismatch for one target must not prevent scoring the other targets";
}

TEST_F(MapsComparison, DegenerateBoundariesOriginReturnsErrorForThatTarget) {
    // min_x == max_x: degenerate on the x axis.
    const MappingBounds degenerate = bounds(50, 50, 0, 50, 0, 50);
    FakeMap3D origin{MapConfig{degenerate, Position3D{}, resolution10()}};
    const auto target = makeUniformMap(bounds(0, 50, 0, 50, 0, 50), VoxelOccupancy::Empty);

    const std::vector<IMap3D*> targets{target.get()};
    const auto scores = simulator::MapsComparison::compare(origin, targets);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 when origin has degenerate boundaries (min_x >= max_x)";
}

TEST_F(MapsComparison, DegenerateBoundariesTargetReturnsErrorForThatTarget) {
    // min_height == max_height: degenerate on the height axis.
    const MappingBounds degenerate = bounds(0, 50, 0, 50, 50, 50);
    const auto origin = makeUniformMap(bounds(0, 50, 0, 50, 0, 50), VoxelOccupancy::Empty);
    FakeMap3D target{MapConfig{degenerate, Position3D{}, resolution10()}};

    const auto scores = simulator::MapsComparison::compare(*origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0)
        << "compare() must return -1.0 when a target has degenerate boundaries "
           "(min_height >= max_height)";
}

// ── multi-target API behavior ─────────────────────────────────────────────────

TEST_F(MapsComparison, MultiTargetReturnsOneScorePerTargetInOrder) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D identical_target{config, VoxelOccupancy::Occupied};
    FakeMap3D different_target{config, VoxelOccupancy::Empty};
    FakeMap3D unmapped_target{config, VoxelOccupancy::Unmapped};

    const auto scores = simulator::MapsComparison::compare(
        origin, {&identical_target, &different_target, &unmapped_target});

    ASSERT_EQ(scores.size(), 3u) << "compare() must return exactly one score per target";
    EXPECT_DOUBLE_EQ(scores[0], 100.0) << "First target (identical) must score 100";
    EXPECT_DOUBLE_EQ(scores[1], 0.0) << "Second target (clearly different) must score 0";
    EXPECT_DOUBLE_EQ(scores[2], 0.0) << "Third target (fully Unmapped) must score 0";
}

TEST_F(MapsComparison, NullTargetReturnsNegativeOneOnlyForThatTarget) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D identical_target{config, VoxelOccupancy::Occupied};
    FakeMap3D different_target{config, VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(
        origin, {&identical_target, nullptr, &different_target});

    ASSERT_EQ(scores.size(), 3u) << "A null target must still occupy a slot in the result";
    EXPECT_DOUBLE_EQ(scores[0], 100.0) << "The target before the null one must be scored normally";
    EXPECT_DOUBLE_EQ(scores[1], -1.0) << "A null target pointer must score -1.0";
    EXPECT_DOUBLE_EQ(scores[2], 0.0)
        << "A null target must not prevent scoring the targets that follow it";
}

// ── scoring correctness ────────────────────────────────────────────────────

TEST_F(MapsComparison, IdenticalMapsReturnHundred) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D target{config, VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "Comparing a target against an identical origin must yield a perfect score";
}

TEST_F(MapsComparison, IdenticalEmptyMapsReturnHundred) {
    // The equality match path (scoreVoxel: `origin_value == target_value`) must not be special-
    // cased to Occupied alone -- a bug like `if (origin == Occupied && target == Occupied)` would
    // pass IdenticalMapsReturnHundred above but wrongly score an identical pair of Empty voxels as
    // a mismatch.
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Empty};
    FakeMap3D target{config, VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "An identical Empty/Empty pair must score 100, the same as an identical Occupied pair";
}

TEST_F(MapsComparison, ClearlyDifferentScoreIsZero) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D target{config, VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0)
        << "A fully-Occupied origin compared against a fully-Empty target over the same "
           "region must mismatch everywhere, scoring 0 (not an error)";
}

TEST_F(MapsComparison, FullyUnmappedTargetReturnsZero) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D target{config, VoxelOccupancy::Unmapped};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0)
        << "A fully-Unmapped target must score 0 against a fully-Occupied origin: origin is "
           "known everywhere, so every Unmapped target voxel counts as a mismatch";
}

TEST_F(MapsComparison, NonOverlappingBoundariesReturnZero) {
    FakeMap3D origin{configFor(bounds(0, 50, 0, 50, 0, 50)), VoxelOccupancy::Occupied};
    FakeMap3D target{configFor(bounds(100, 150, 100, 150, 100, 150)), VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0)
        << "An origin and a target with completely non-overlapping boundaries must return "
           "0.0 (an empty intersection is valid, not an error)";
}

TEST_F(MapsComparison, IntersectingBoundariesWithNoUsableGroundTruthReturnZero) {
    // Boundaries overlap (non-empty WorldIntersection), but every voxel in the intersection is
    // Unmapped on the origin side, so total_compared stays 0 -- distinct from the
    // NonOverlappingBoundariesReturnZero case above, which returns 0.0 via the empty-intersection
    // path before any voxel is even sampled. This pins down the *other* total_compared==0
    // codepath: scoring zero comparable positions must still report 0.0, not a perfect 100.0.
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Unmapped};
    FakeMap3D target{config, VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0)
        << "When boundaries overlap but no voxel has usable ground truth (origin Unmapped "
           "everywhere), total_compared is 0; this degenerate case must score 0.0, not 100.0";
}

// ── different physical boundaries (cross-size scenarios) ─────────────────────

TEST_F(MapsComparison, SubregionIntersectionScoresCorrectly) {
    // Origin map covers region A ([0,50]) plus region B ([50,100]) along x.
    // Target map covers only region A.
    const MappingBounds origin_bounds = bounds(0, 100, 0, 50, 0, 50);
    const MappingBounds target_bounds = bounds(0, 50, 0, 50, 0, 50);

    // Region A (== target_bounds) reads Occupied, matching the target map's content exactly;
    // region B reads Empty but is outside the intersection, so it must not affect the score.
    // Content is computed by a position rule (not real Map3DImpl storage), so this test stays
    // isolated from Map3DImpl's own (separately tested) set()/atVoxel() behavior.
    RuleMap3D origin{configFor(origin_bounds), [target_bounds](const Position3D& p) {
                          return withinBounds(p, target_bounds) ? VoxelOccupancy::Occupied
                                                                 : VoxelOccupancy::Empty;
                      }};
    FakeMap3D target{configFor(target_bounds), VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "Only the intersection (region A) should be compared; region B, which is "
           "outside the target map's boundaries, must not affect the score";
}

TEST_F(MapsComparison, SubregionWithUnmappedVoxelsScoresBelow100) {
    const MappingBounds origin_bounds = bounds(0, 100, 0, 50, 0, 50);
    const MappingBounds target_bounds = bounds(0, 50, 0, 50, 0, 50);

    RuleMap3D origin{configFor(origin_bounds), [target_bounds](const Position3D& p) {
                          return withinBounds(p, target_bounds) ? VoxelOccupancy::Occupied
                                                                 : VoxelOccupancy::Empty;
                      }};
    // Occupied everywhere in target_bounds, except one voxel within the intersection left Unmapped.
    const Position3D unmapped_voxel = pos(0, 0, 0);
    RuleMap3D target{configFor(target_bounds), [unmapped_voxel](const Position3D& p) {
                          return (p.x == unmapped_voxel.x && p.y == unmapped_voxel.y &&
                                  p.z == unmapped_voxel.z)
                                     ? VoxelOccupancy::Unmapped
                                     : VoxelOccupancy::Occupied;
                      }};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_LT(scores[0], 100.0) << "An Unmapped voxel in the target map within the "
                                    "intersection must count as a mismatch, not a match";
    EXPECT_GT(scores[0], 0.0) << "Only one voxel out of many is Unmapped; the score should "
                                  "be close to, but below, 100";
}

TEST_F(MapsComparison, WorldCoordinateComparisonNotIndexBased) {
    // origin: world x in [0,50), offset.x == 0  -> array index i <-> world x = i * 10cm.
    const MappingBounds origin_bounds = bounds(0, 50, 0, 10, 0, 10);
    // target: world x in [20,70), offset.x == 20 -> array index i <-> world x = 20 + i * 10cm.
    const MappingBounds target_bounds = bounds(20, 70, 0, 10, 0, 10);

    // Both maps share one world-position rule (Occupied for world_x < 35cm, Empty otherwise),
    // expressed directly against world coordinates rather than through Map3DImpl storage/indexing.
    // The intersection (world x in [20,50)) would be array indices 2-4 in a real origin map but
    // indices 0-2 in a real target map -- a raw index-by-index comparison would compare unrelated
    // voxels, while a world-coordinate comparison must align them correctly. Using a position rule
    // directly keeps this test focused on that alignment behavior, not on whether Map3DImpl's own
    // indexing happens to be correct.
    const auto occupiedBelowX35 = [](const Position3D& p) {
        return p.x.force_numerical_value_in(cm) < 35.0 ? VoxelOccupancy::Occupied : VoxelOccupancy::Empty;
    };
    RuleMap3D origin{configFor(origin_bounds), occupiedBelowX35};
    RuleMap3D target{configFor(target_bounds), occupiedBelowX35};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "compare() must align voxels by world position, not by raw NpyArray index";
}

// ── non-exact-multiple sampling ───────────────────────────────────────────────

TEST_F(MapsComparison, NonExactMultipleBoundariesSampledCorrectly) {
    // x-span of 15cm at 10cm resolution is not an exact multiple of the step:
    // ceil(15/10) = 2 sample positions along x (world x = 0 and 10).
    const MappingBounds b = bounds(0, 15, 0, 10, 0, 10);
    const MapConfig config = configFor(b);

    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    // Occupied at world x=0, Empty at world x=10 (the only two positions this span can sample).
    RuleMap3D target{config, [](const Position3D& p) {
                          return p.x.force_numerical_value_in(cm) < 5.0 ? VoxelOccupancy::Occupied
                                                                        : VoxelOccupancy::Empty;
                      }};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 50.0)
        << "a 15cm span at 10cm resolution must sample two x positions (0 and 10cm); "
           "with one match and one mismatch among them, the expected score is 50.0";
}

// ── Unmapped semantics ────────────────────────────────────────────────────────

TEST_F(MapsComparison, OriginUnmappedIsSkipped) {
    // 10 voxels along x (and a single voxel in y and z).
    const MappingBounds b = bounds(0, 100, 0, 10, 0, 10);
    const MapConfig config = configFor(b);

    // x=0..40 (5 positions) are Occupied; x=50..90 (5 positions) are Unmapped.
    RuleMap3D origin{config, [](const Position3D& p) {
                          return p.x.force_numerical_value_in(cm) < 50.0 ? VoxelOccupancy::Occupied
                                                                         : VoxelOccupancy::Unmapped;
                      }};
    FakeMap3D target{config, VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "Positions where the origin map is Unmapped must be skipped entirely; only "
           "the 5 positions with origin=Occupied are counted, and all match.";
}

TEST_F(MapsComparison, OriginAndTargetBothUnmappedIsSkipped) {
    const MappingBounds b = bounds(0, 100, 0, 10, 0, 10);
    const MapConfig config = configFor(b);

    const auto occupiedBelowX50OtherwiseUnmapped = [](const Position3D& p) {
        return p.x.force_numerical_value_in(cm) < 50.0 ? VoxelOccupancy::Occupied
                                                        : VoxelOccupancy::Unmapped;
    };
    RuleMap3D origin{config, occupiedBelowX50OtherwiseUnmapped};
    RuleMap3D target{config, occupiedBelowX50OtherwiseUnmapped};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "x=0..40: origin=Occupied, target=Occupied -> 5 matches. x=50..90: "
           "origin=Unmapped -> skipped (the origin check fires before the target check).";
}

TEST_F(MapsComparison, PartiallyMappedTargetScoresProportionally) {
    // 10 voxels along x (and a single voxel in y and z).
    const MappingBounds b = bounds(0, 100, 0, 10, 0, 10);
    const MapConfig config = configFor(b);

    FakeMap3D origin{config, VoxelOccupancy::Occupied};

    // Match the origin map for the first half (world x in [0,50)); leave the
    // second half (world x in [50,100)) Unmapped.
    RuleMap3D target{config, [](const Position3D& p) {
                          return p.x.force_numerical_value_in(cm) < 50.0 ? VoxelOccupancy::Occupied
                                                                         : VoxelOccupancy::Unmapped;
                      }};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 50.0)
        << "origin is always Occupied (never Unmapped), so the 5 target-Unmapped "
           "positions (x=50..90) count as mismatches against a known origin value: "
           "5 matches out of 10 = 50.0";
}

// ── PotentiallyOccupied scoring ─────────────────────────────────────────────

TEST_F(MapsComparison, OccupiedVsPotentiallyOccupiedScoresHalfCredit) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    FakeMap3D target{config, VoxelOccupancy::PotentiallyOccupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GT(scores[0], 0.0);
    EXPECT_LT(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[0], 50.0)
        << "Occupied vs PotentiallyOccupied is a partial match, "
           "so the score must be 50.0, not 100.0";
}

TEST_F(MapsComparison, PotentiallyOccupiedVsOccupiedScoresHalfCredit) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::PotentiallyOccupied};
    FakeMap3D target{config, VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GT(scores[0], 0.0);
    EXPECT_LT(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[0], 50.0)
        << "PotentiallyOccupied vs Occupied is a partial match, "
           "so the score must be 50.0, not 100.0";
}

TEST_F(MapsComparison, PotentiallyOccupiedVsPotentiallyOccupiedScoresFullCredit) {
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::PotentiallyOccupied};
    FakeMap3D target{config, VoxelOccupancy::PotentiallyOccupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "An exact PotentiallyOccupied match is a full match";
}

TEST_F(MapsComparison, PotentiallyOccupiedVsEmptyScoresZeroNotPartialCredit) {
    // The half-credit rule above is specifically the (Occupied, PotentiallyOccupied) pair in
    // either order -- not "any mismatch involving PotentiallyOccupied". A bug that broadens the
    // partial-credit condition (e.g. `origin_value == PotentiallyOccupied || target_value ==
    // PotentiallyOccupied`) would wrongly give this pair credit too.
    const MapConfig config = configFor(bounds(0, 50, 0, 50, 0, 50));
    FakeMap3D origin{config, VoxelOccupancy::PotentiallyOccupied};
    FakeMap3D target{config, VoxelOccupancy::Empty};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0)
        << "PotentiallyOccupied vs Empty is a full mismatch (0.0), not the 50.0 partial credit "
           "reserved for the (Occupied, PotentiallyOccupied) pair";
}

// ── OutOfBounds within the computed intersection ─────────────────────────────
//
// scoreVoxel() treats a position that reads as OutOfBounds on either map as skipped (not counted
// toward the score at all) -- distinct from Unmapped, which is skipped only on the *origin* side
// but counted as a mismatch on the *target* side. This should not normally happen for a position
// inside compare()'s own computed intersection, but the code defends against it explicitly (e.g.
// a map whose getMapConfig() boundaries don't match its real storage extent), so it is worth
// pinning down -- especially the target side, which is easy to conflate with "target Unmapped"
// since both represent "unknown" but score completely differently.

TEST_F(MapsComparison, TargetOutOfBoundsWithinIntersectionIsSkippedNotCountedAsMismatch) {
    const MappingBounds b = bounds(0, 30, 0, 10, 0, 10);
    const MapConfig config = configFor(b);
    const Position3D out_of_bounds_voxel = pos(0, 0, 0);

    FakeMap3D origin{config, VoxelOccupancy::Occupied};
    RuleMap3D target{config, [out_of_bounds_voxel](const Position3D& p) {
                          return (p.x == out_of_bounds_voxel.x && p.y == out_of_bounds_voxel.y &&
                                  p.z == out_of_bounds_voxel.z)
                                     ? VoxelOccupancy::OutOfBounds
                                     : VoxelOccupancy::Occupied;
                      }};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "a target position that reads OutOfBounds must be skipped, not counted as a mismatch "
           "the way a target-Unmapped position would be";
}

TEST_F(MapsComparison, OriginOutOfBoundsWithinIntersectionIsSkipped) {
    const MappingBounds b = bounds(0, 30, 0, 10, 0, 10);
    const MapConfig config = configFor(b);
    const Position3D out_of_bounds_voxel = pos(0, 0, 0);

    RuleMap3D origin{config, [out_of_bounds_voxel](const Position3D& p) {
                          return (p.x == out_of_bounds_voxel.x && p.y == out_of_bounds_voxel.y &&
                                  p.z == out_of_bounds_voxel.z)
                                     ? VoxelOccupancy::OutOfBounds
                                     : VoxelOccupancy::Occupied;
                      }};
    FakeMap3D target{config, VoxelOccupancy::Occupied};

    const auto scores = simulator::MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0)
        << "an origin position that reads OutOfBounds must be skipped from the score entirely";
}
