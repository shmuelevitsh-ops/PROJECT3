#pragma once

// Shared sphere-vs-AABB collision geometry.

#include <Common/Units.h>

#include <algorithm>

namespace user_common_322889890_315113738 {

// Returns true when the sphere intersects or touches the axis-aligned box.
[[nodiscard]] inline bool sphereIntersectsAxisAlignedBox(
    const common::Position3D& center,
    common::PhysicalLength radius,
    const common::Position3D& box_min,
    const common::Position3D& box_max) {
    using common::cm;

    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);
    const double r = radius.force_numerical_value_in(cm);

    const double closest_x = std::clamp(
        cx, box_min.x.force_numerical_value_in(cm), box_max.x.force_numerical_value_in(cm));
    const double closest_y = std::clamp(
        cy, box_min.y.force_numerical_value_in(cm), box_max.y.force_numerical_value_in(cm));
    const double closest_z = std::clamp(
        cz, box_min.z.force_numerical_value_in(cm), box_max.z.force_numerical_value_in(cm));

    const double dx = cx - closest_x;
    const double dy = cy - closest_y;
    const double dz = cz - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= r * r;
}

} // namespace user_common_322889890_315113738
