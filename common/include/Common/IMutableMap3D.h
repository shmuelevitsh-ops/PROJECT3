#pragma once

#include <Common/IMap3D.h>

#include <filesystem>

namespace common {

class IMutableMap3D : public IMap3D {
public:
    ~IMutableMap3D() override = default;
    virtual void set(const Position3D& pos, types::VoxelOccupancy value) = 0;
    virtual void save(const std::filesystem::path& path) const = 0;
};

} // namespace common
