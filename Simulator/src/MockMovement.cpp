#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace simulator {

namespace types = common::types;
namespace si = common::si;
namespace mp = common::mp;

using common::HorizontalAngle;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

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

    const double dir_x =
        si::cos(heading).force_numerical_value_in(mp::one);
    const double dir_y =
        si::sin(heading).force_numerical_value_in(mp::one);
    const double distance_cm =
        distance.force_numerical_value_in(cm);

    const Position3D next_pos{
        current_pos.x + dir_x * distance_cm * x_extent[cm],
        current_pos.y + dir_y * distance_cm * y_extent[cm],
        current_pos.z,
    };

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

    gps_.setPosition(next_pos);
    return types::MovementResult{true, {}};
}

} // namespace simulator