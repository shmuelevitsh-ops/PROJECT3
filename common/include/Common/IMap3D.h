#pragma once

#include <Common/Types.h>

namespace common {

class IMap3D {
public:
    virtual ~IMap3D() = default;
    [[nodiscard]] virtual types::VoxelOccupancy atVoxel(const Position3D& pos) const = 0;
    [[nodiscard]] virtual types::MapConfig getMapConfig() const = 0;
    [[nodiscard]] virtual bool isInBounds(const Position3D& pos) const = 0;
};

} // namespace common
