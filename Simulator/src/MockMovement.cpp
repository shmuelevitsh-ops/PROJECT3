#include <Simulator/MockMovement.h>

#include <Simulator/SimulationException.h>

#include <UserCommon/AdvanceDirection.h>
#include <UserCommon/SphereAabbCollision.h>

#include <cmath>

namespace simulator {

namespace types = common::types;

using common::HorizontalAngle;
using common::IMap3D;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// Fraction of one voxel's resolution used as the step size when sampling a movement path for
// collision detection (in addition to the exact destination, which is always checked separately).
constexpr double kPathSampleStepFraction = 0.1;

[[nodiscard]] double numCm(PhysicalLength v) { return v.force_numerical_value_in(cm); }

// Checks whether the drone sphere intersects an Occupied voxel.
[[nodiscard]] bool sphereHitsWall(const IMap3D& map, const Position3D& center, PhysicalLength radius) {
    const types::MapConfig config = map.getMapConfig();
    const double resolution_cm = numCm(config.resolution);
    if (!(resolution_cm > 0.0)) {
        return false;
    }
    const double radius_cm = numCm(radius);

    const double cx = (center.x + config.offset.x).force_numerical_value_in(cm);
    const double cy = (center.y + config.offset.y).force_numerical_value_in(cm);
    const double cz = (center.z + config.offset.z).force_numerical_value_in(cm);

    // ceil(...)-1 includes the lower voxel when the sphere exactly touches
// a voxel boundary; the upper bound is already handled correctly by floor().
    const long ix_min = static_cast<long>(std::ceil((cx - radius_cm) / resolution_cm)) - 1;
    const long ix_max = static_cast<long>(std::floor((cx + radius_cm) / resolution_cm));
    const long iy_min = static_cast<long>(std::ceil((cy - radius_cm) / resolution_cm)) - 1;
    const long iy_max = static_cast<long>(std::floor((cy + radius_cm) / resolution_cm));
    const long iz_min = static_cast<long>(std::ceil((cz - radius_cm) / resolution_cm)) - 1;
    const long iz_max = static_cast<long>(std::floor((cz + radius_cm) / resolution_cm));

    for (long ix = ix_min; ix <= ix_max; ++ix) {
        for (long iy = iy_min; iy <= iy_max; ++iy) {
            for (long iz = iz_min; iz <= iz_max; ++iz) {
                const Position3D voxel_min{
                    static_cast<double>(ix) * resolution_cm * x_extent[cm] - config.offset.x,
                    static_cast<double>(iy) * resolution_cm * y_extent[cm] - config.offset.y,
                    static_cast<double>(iz) * resolution_cm * z_extent[cm] - config.offset.z,
                };
                const Position3D voxel_max{
                    voxel_min.x + resolution_cm * x_extent[cm],
                    voxel_min.y + resolution_cm * y_extent[cm],
                    voxel_min.z + resolution_cm * z_extent[cm],
                };
                if (!user_common_322889890_315113738::sphereIntersectsAxisAlignedBox(
                        center, radius, voxel_min, voxel_max)) {
                    continue;
                }
                const Position3D voxel_center{
                    voxel_min.x + 0.5 * resolution_cm * x_extent[cm],
                    voxel_min.y + 0.5 * resolution_cm * y_extent[cm],
                    voxel_min.z + 0.5 * resolution_cm * z_extent[cm],
                };
                if (map.atVoxel(voxel_center) == types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    return false;
}

// Samples the movement path and checks the exact destination for collisions.
[[nodiscard]] bool pathCollides(const IMap3D& map, const Position3D& start, const Position3D& destination,
                                 PhysicalLength radius) {
    const double resolution_cm = numCm(map.getMapConfig().resolution);
    const double dx = (destination.x - start.x).force_numerical_value_in(cm);
    const double dy = (destination.y - start.y).force_numerical_value_in(cm);
    const double dz = (destination.z - start.z).force_numerical_value_in(cm);
    const double total_distance_cm = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (resolution_cm > 0.0 && total_distance_cm > 0.0) {
        const double step_cm = kPathSampleStepFraction * resolution_cm;
        const long num_steps = static_cast<long>(std::floor(total_distance_cm / step_cm));
        for (long i = 0; i <= num_steps; ++i) {
            const double t = (static_cast<double>(i) * step_cm) / total_distance_cm;
            const Position3D sample{
                start.x + t * dx * x_extent[cm],
                start.y + t * dy * y_extent[cm],
                start.z + t * dz * z_extent[cm],
            };
            if (sphereHitsWall(map, sample, radius)) {
                return true;
            }
        }
    }

    // Always check the exact destination too, even when it is not an exact multiple of the
    // sampling step.
    return sphereHitsWall(map, destination, radius);
}

} // namespace

MockMovement::MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(hidden_map), drone_radius_(drone_radius) {}

types::MovementResult MockMovement::rotate(
    types::RotationDirection direction,
    HorizontalAngle angle) {

    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;

    gps_.setHeading(
        Orientation{current.horizontal + signed_angle, current.altitude});

    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const HorizontalAngle heading = gps_.heading().horizontal;

    // Uses the same Advance direction calculation as DroneControlImpl.
    const user_common_322889890_315113738::AdvanceDirection direction =
        user_common_322889890_315113738::advanceDirection(heading);
    const double distance_cm =
        distance.force_numerical_value_in(cm);

    const Position3D next_pos{
        current_pos.x + direction.x * distance_cm * x_extent[cm],
        current_pos.y + direction.y * distance_cm * y_extent[cm],
        current_pos.z,
    };

    if (pathCollides(hidden_map_, current_pos, next_pos, drone_radius_)) {
        throw SimulationException(
            "MOVEMENT_COLLISION",
            "MockMovement::advance: MOVEMENT_COLLISION - drone sphere intersects an occupied "
            "voxel (wall) along the advance path; the drone was not moved.");
    }

    gps_.setPosition(next_pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const double distance_cm =
        distance.force_numerical_value_in(cm);

    const Position3D next_pos{
        current_pos.x,
        current_pos.y,
        current_pos.z + distance_cm * z_extent[cm],
    };

    if (pathCollides(hidden_map_, current_pos, next_pos, drone_radius_)) {
        throw SimulationException(
            "MOVEMENT_COLLISION",
            "MockMovement::elevate: MOVEMENT_COLLISION - drone sphere intersects an occupied "
            "voxel (wall) along the elevate path; the drone was not moved.");
    }

    gps_.setPosition(next_pos);
    return types::MovementResult{true, {}};
}

} // namespace simulator
