#include <Simulator/MockLidar.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <limits>

namespace simulator {
namespace types = common::types;
namespace si = common::si;
namespace mp = common::mp;

using common::AltitudeAngle;
using common::HorizontalAngle;
using common::IGPS;
using common::IMap3D;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps)
    : config_(config), map_(map), gps_(gps) {}

types::LidarConfigData MockLidar::config() const{
    return config_;
}

types::LidarScanResult MockLidar::scan(Orientation scan_orientation) const {
    types::LidarScanResult results;
    if (config_.fov_circles == 0) {
        return results;
    }

    const Orientation sensor_heading = gps_.heading();
    const Orientation center_beam_abs{
        scan_orientation.horizontal + sensor_heading.horizontal,
        scan_orientation.altitude + sensor_heading.altitude,
    };

    const PhysicalLength center_distance = traceBeam(center_beam_abs);
    results.push_back(types::LidarHit{center_distance, scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg; // explicit static cast for proofing
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.z_min),
                altitude_delta(altitude_offset, config_.z_min),
            };
            const Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const Orientation absolute_beam{
                relative_beam.horizontal + sensor_heading.horizontal,
                relative_beam.altitude + sensor_heading.altitude,
            };
            const PhysicalLength distance = traceBeam(absolute_beam);
            results.push_back(types::LidarHit{distance, relative_beam});
        }
    }

    return results;
}

PhysicalLength MockLidar::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = gps_.position();
    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    // step based on size of the map's resolution
    const PhysicalLength step = 0.1 * map_.getMapConfig().resolution;

    for (PhysicalLength distance = 0.0 * cm; distance <= config_.z_max; distance += step) {
        // Computing target voxel position
        const double distance_cm = distance.force_numerical_value_in(cm);
        const double dir_x = dx.force_numerical_value_in(mp::one);
        const double dir_y = dy.force_numerical_value_in(mp::one);
        const double dir_z = dz.force_numerical_value_in(mp::one);

        const Position3D sample{
            origin.x + dir_x * distance_cm * x_extent[cm],
            origin.y + dir_y * distance_cm * y_extent[cm],
            origin.z + dir_z * distance_cm * z_extent[cm],
        };
        if (map_.atVoxel(sample) == types::VoxelOccupancy::Occupied) {
            if (distance < config_.z_min) {
                return 0.0 * cm;
            }
            return distance;
        }
    }
    // Beam never returns
    return std::numeric_limits<double>::max() * cm;
}

} // namespace simulator
