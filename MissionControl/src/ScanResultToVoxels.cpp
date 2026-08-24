#include <MissionControl/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <limits>

namespace mission_control_322889890_315113738 {

namespace common_types = common::types;
namespace mp = common::mp;
namespace si = common::si;

using common::IMutableMap3D;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

[[nodiscard]] bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

[[nodiscard]] bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) ==
           std::numeric_limits<double>::max();
}

// LidarHit angles are relative to the scan direction. The map update needs a
// world-facing beam direction, so we add the drone heading.
[[nodiscard]] Orientation absoluteBeamOrientation(
    const Orientation& drone_heading,
    const Orientation& relative_beam) {

    return Orientation{
        relative_beam.horizontal + drone_heading.horizontal,
        relative_beam.altitude + drone_heading.altitude};
}

// Converts a distance along a beam into a world-space position.
[[nodiscard]] Position3D pointAlongBeam(
    const Position3D& origin,
    const Orientation& beam_orientation,
    PhysicalLength distance) {

    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);

    return Position3D{
        origin.x + dir_x * distance_cm * x_extent[cm],
        origin.y + dir_y * distance_cm * y_extent[cm],
        origin.z + dir_z * distance_cm * z_extent[cm]};
}

// Evidence strength for conflicting writes to the same voxel.
[[nodiscard]] int occupancyPriority(common_types::VoxelOccupancy occupancy) {
    switch (occupancy) {
        case common_types::VoxelOccupancy::Occupied:
            return 3;
        case common_types::VoxelOccupancy::Empty:
            return 2;
        case common_types::VoxelOccupancy::PotentiallyOccupied:
            return 1;
        case common_types::VoxelOccupancy::Unmapped:
        case common_types::VoxelOccupancy::OutOfBounds:
            return 0;
    }

    return 0;
}

void setIfStronger(IMutableMap3D& output_map,
                   const Position3D& position,
                   common_types::VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) {
        return;
    }

    const common_types::VoxelOccupancy current_value =
        output_map.atVoxel(position);

    if (occupancyPriority(value) > occupancyPriority(current_value)) {
        output_map.set(position, value);
    }
}

// Walks a beam segment and applies the same observation to each sampled point.
void markBeamSegment(IMutableMap3D& output_map,
                     const Position3D& scan_origin,
                     const Orientation& beam_orientation,
                     PhysicalLength start_distance,
                     PhysicalLength end_distance,
                     PhysicalLength step,
                     common_types::VoxelOccupancy value) {
    for (PhysicalLength distance = start_distance;
         distance <= end_distance;
         distance += step) {

        const Position3D current_point =
            pointAlongBeam(scan_origin, beam_orientation, distance);

        if (!output_map.isInBounds(current_point)) {
            break;
        }

        setIfStronger(output_map, current_point, value);
    }
}

} // namespace

void ScanResultToVoxels::applyToMap(
    IMutableMap3D& output_map,
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const common_types::LidarScanResult& scan,
    const common_types::LidarConfigData& lidar_config) {

    if (!output_map.isInBounds(scan_origin)) {
        return;
    }

    const PhysicalLength step =
        0.1 * output_map.getMapConfig().resolution;

    if (step <= 0.0 * cm) {
        return;
    }

    for (const common_types::LidarHit& hit : scan) {
        const Orientation beam_orientation =
            absoluteBeamOrientation(drone_heading, hit.angle);

        if (isZeroDistance(hit.distance)) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                lidar_config.z_min,
                step,
                common_types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (isMissDistance(hit.distance)) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                lidar_config.z_max,
                step,
                common_types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                hit.distance,
                step,
                common_types::VoxelOccupancy::Empty);

            setIfStronger(
                output_map,
                pointAlongBeam(
                    scan_origin,
                    beam_orientation,
                    hit.distance),
                common_types::VoxelOccupancy::Occupied);
        }
    }
}

} // namespace mission_control_322889890_315113738