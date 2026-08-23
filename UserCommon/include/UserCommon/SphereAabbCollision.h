#pragma once

// Pure sphere-vs-axis-aligned-bounding-box geometry, shared between Algorithm's own safety
// checks (MappingAlgorithmImpl.cpp) and Simulator's MockMovement collision detection so both
// apply the identical closest-point-on-AABB test. Header-only, no map access, no
// VoxelOccupancy policy, no runtime (.so) dependency: including this file does not stop
// Algorithm.so (or MissionControl.so, if it ever uses this) from being built and used
// independently.

#include <Common/Units.h>

#include <algorithm>

namespace user_common_322889890_315113738 {

// True if a sphere of `radius` centered at `center` intersects (including merely touching) the
// axis-aligned box spanning [box_min, box_max] on every axis. Uses the closest point on the box
// to `center`, not the box's own center -- so a box is correctly reported as intersecting even
// when its center lies farther from `center` than `radius`, as long as a corner/edge/face is
// still within reach.
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
