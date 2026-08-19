#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si/unit_symbols.h>

namespace common {

namespace mp = mp_units;
namespace isq = mp_units::isq;
namespace si = mp_units::si;
using mp_units::si::unit_symbols::deg;
using si::unit_symbols::cm;

QUANTITY_SPEC(x_extent, isq::length);
QUANTITY_SPEC(y_extent, isq::length);
QUANTITY_SPEC(z_extent, isq::length);

using PhysicalLength = mp::quantity<isq::length[cm], double>;
using XLength = mp::quantity<x_extent[cm], double>;
using YLength = mp::quantity<y_extent[cm], double>;
using ZLength = mp::quantity<z_extent[cm], double>;

struct Position3D {
    XLength x{};
    YLength y{};
    ZLength z{};
};

QUANTITY_SPEC(horizontal_angle, isq::angular_measure);
QUANTITY_SPEC(altitude_angle, isq::angular_measure);

using HorizontalAngle = mp::quantity<horizontal_angle[deg], double>;
using AltitudeAngle = mp::quantity<altitude_angle[deg], double>;

struct Orientation {
    HorizontalAngle horizontal{};
    AltitudeAngle altitude{};
};

[[nodiscard]] constexpr Position3D operator+(const Position3D& lhs, const Position3D& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

[[nodiscard]] constexpr Position3D operator-(const Position3D& lhs, const Position3D& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

} // namespace common
