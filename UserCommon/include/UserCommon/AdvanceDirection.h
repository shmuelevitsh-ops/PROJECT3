#pragma once

// Shared Advance-direction math used by DroneControlImpl and MockMovement.

#include <Common/Units.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace user_common_322889890_315113738 {

// Removes floating-point residue from axis-aligned directions.
inline constexpr double kNegligibleDirectionComponent = 1e-9;

[[nodiscard]] inline double zeroIfNegligibleDirectionComponent(double v) {
    return std::abs(v) < kNegligibleDirectionComponent ? 0.0 : v;
}

// The unit (dir_x, dir_y) horizontal direction an Advance travels along for `heading`, with any
// merely-floating-point-noise component from an axis-aligned heading zeroed out.
struct AdvanceDirection {
    double x = 0.0;
    double y = 0.0;
};

[[nodiscard]] inline AdvanceDirection advanceDirection(common::HorizontalAngle heading) {
    namespace si = common::si;
    namespace mp = common::mp;
    return AdvanceDirection{
        zeroIfNegligibleDirectionComponent(si::cos(heading).force_numerical_value_in(mp::one)),
        zeroIfNegligibleDirectionComponent(si::sin(heading).force_numerical_value_in(mp::one)),
    };
}

} // namespace user_common_322889890_315113738
