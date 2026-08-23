#pragma once

// Shared horizontal-direction math for an Advance movement, used by both MissionControl's
// DroneControlImpl (to predict where an Advance chunk should land, for Optional Common-Issues
// rows 10/12) and Simulator's MockMovement (to actually apply one). Both sides must compute the
// exact same (dir_x, dir_y) for a given heading -- if DroneControlImpl zeroed a negligible
// direction residue that MockMovement's own advance() left in as a real (if minuscule) component,
// the two would disagree about where the drone physically ended up purely from using different
// movement math, not from any real GPS inconsistency. Header-only, no map access, no runtime
// (.so) dependency, mirroring UserCommon/SphereAabbCollision.h's sharing pattern.

#include <Common/Units.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace user_common_322889890_315113738 {

// cos/sin of an angle that is conceptually parallel to an axis (e.g. heading == 90deg) still
// yield a tiny nonzero double (cos(90deg) ~ 6e-17) due to floating-point rounding, not a real
// directional component. Below this magnitude, a direction component is treated as exactly zero
// so it never imposes a spurious boundary constraint on that axis, nor a spurious displacement
// along it.
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
