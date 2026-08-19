#include <Simulator/MapsComparison.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>

namespace simulator {

namespace common_types = common::types;

using common::IMap3D;
using common::Position3D;
using common::XLength;
using common::YLength;
using common::ZLength;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// Partial credit for uncertainty next to a definite obstacle.
constexpr double kPotentiallyOccupiedPartialCredit = 0.5;

// Number of sample points covering [0, span_cm) at the given step size.
[[nodiscard]] long sampleCount(double span_cm, double step_cm) {
    if (span_cm <= 0.0 || step_cm <= 0.0) {
        return 0;
    }
    return static_cast<long>(std::ceil(span_cm / step_cm));
}

// True if every axis of `bounds` is non-degenerate (min < max).
[[nodiscard]] bool boundariesAreValid(const common_types::MappingBounds& bounds) {
    return bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y &&
           bounds.min_height < bounds.max_height;
}

// Returns -1.0 if the maps cannot be compared, or nullopt otherwise.
[[nodiscard]] std::optional<double> validateComparable(
    const common_types::MapConfig& origin_cfg,
    const common_types::MapConfig& target_cfg) {

    if (!(origin_cfg.resolution.force_numerical_value_in(cm) > 0.0)) {
        std::cerr << "MapsComparison: origin has non-positive resolution.\n";
        return -1.0;
    }
    if (!(target_cfg.resolution.force_numerical_value_in(cm) > 0.0)) {
        std::cerr << "MapsComparison: target has non-positive resolution.\n";
        return -1.0;
    }
    if (origin_cfg.resolution != target_cfg.resolution) {
        std::cerr << "MapsComparison: origin and target have different resolutions.\n";
        return -1.0;
    }
    if (!boundariesAreValid(origin_cfg.boundaries)) {
        std::cerr << "MapsComparison: origin has degenerate boundaries.\n";
        return -1.0;
    }
    if (!boundariesAreValid(target_cfg.boundaries)) {
        std::cerr << "MapsComparison: target has degenerate boundaries.\n";
        return -1.0;
    }

    return std::nullopt;
}

struct WorldIntersection {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_z{};
    ZLength max_z{};
};

[[nodiscard]] WorldIntersection intersectBoundaries(
    const common_types::MappingBounds& origin_bounds,
    const common_types::MappingBounds& target_bounds) {

    return WorldIntersection{
        std::max(origin_bounds.min_x, target_bounds.min_x),
        std::min(origin_bounds.max_x, target_bounds.max_x),
        std::max(origin_bounds.min_y, target_bounds.min_y),
        std::min(origin_bounds.max_y, target_bounds.max_y),
        std::max(origin_bounds.min_height, target_bounds.min_height),
        std::min(origin_bounds.max_height, target_bounds.max_height)};
}

[[nodiscard]] bool isEmpty(const WorldIntersection& world) {
    return world.min_x >= world.max_x ||
           world.min_y >= world.max_y ||
           world.min_z >= world.max_z;
}

struct VoxelScore {
    double contribution = 0.0;
    bool compared = false;
};

[[nodiscard]] VoxelScore scoreVoxel(
    common_types::VoxelOccupancy origin_value,
    common_types::VoxelOccupancy target_value) {

    if (origin_value == common_types::VoxelOccupancy::OutOfBounds ||
        target_value == common_types::VoxelOccupancy::OutOfBounds) {
        std::cerr << "MapsComparison: position inside the computed intersection is "
                     "OutOfBounds in one of the maps; skipping.\n";
        return {};
    }

    if (origin_value == common_types::VoxelOccupancy::Unmapped) {
        return {};
    }

    VoxelScore score;
    score.compared = true;

    if (target_value == common_types::VoxelOccupancy::Unmapped) {
        return score;
    }

    if (origin_value == target_value) {
        score.contribution = 1.0;
    } else if (
        (origin_value == common_types::VoxelOccupancy::Occupied &&
         target_value == common_types::VoxelOccupancy::PotentiallyOccupied) ||
        (origin_value == common_types::VoxelOccupancy::PotentiallyOccupied &&
         target_value == common_types::VoxelOccupancy::Occupied)) {
        score.contribution = kPotentiallyOccupiedPartialCredit;
    }

    return score;
}

[[nodiscard]] double compareOne(const IMap3D& origin, const IMap3D& target) {
    const common_types::MapConfig origin_cfg = origin.getMapConfig();
    const common_types::MapConfig target_cfg = target.getMapConfig();

    if (const auto error_score = validateComparable(origin_cfg, target_cfg)) {
        return *error_score;
    }

    const WorldIntersection world =
        intersectBoundaries(origin_cfg.boundaries, target_cfg.boundaries);

    if (isEmpty(world)) {
        return 0.0;
    }

    const double step_cm =
        origin_cfg.resolution.force_numerical_value_in(cm);

    const double min_x_cm =
        world.min_x.force_numerical_value_in(cm);
    const double min_y_cm =
        world.min_y.force_numerical_value_in(cm);
    const double min_z_cm =
        world.min_z.force_numerical_value_in(cm);

    const long count_x = sampleCount(
        (world.max_x - world.min_x).force_numerical_value_in(cm),
        step_cm);
    const long count_y = sampleCount(
        (world.max_y - world.min_y).force_numerical_value_in(cm),
        step_cm);
    const long count_z = sampleCount(
        (world.max_z - world.min_z).force_numerical_value_in(cm),
        step_cm);

    double matches = 0.0;
    long total_compared = 0;

    for (long ix = 0; ix < count_x; ++ix) {
        for (long iy = 0; iy < count_y; ++iy) {
            for (long iz = 0; iz < count_z; ++iz) {
                const Position3D world_pos{
                    (min_x_cm + static_cast<double>(ix) * step_cm) * x_extent[cm],
                    (min_y_cm + static_cast<double>(iy) * step_cm) * y_extent[cm],
                    (min_z_cm + static_cast<double>(iz) * step_cm) * z_extent[cm]};

                const VoxelScore score =
                    scoreVoxel(origin.atVoxel(world_pos), target.atVoxel(world_pos));

                if (score.compared) {
                    ++total_compared;
                    matches += score.contribution;
                }
            }
        }
    }

    if (total_compared == 0) {
        return 0.0;
    }

    return (matches / static_cast<double>(total_compared)) * 100.0;
}

} // namespace

std::vector<double> MapsComparison::compare(
    const IMap3D& origin,
    const std::vector<IMap3D*> targets) {

    std::vector<double> scores;
    scores.reserve(targets.size());

    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            std::cerr << "MapsComparison: null target pointer; "
                         "returning -1.0 for this target.\n";
            scores.push_back(-1.0);
            continue;
        }

        scores.push_back(compareOne(origin, *target));
    }

    return scores;
}

} // namespace simulator