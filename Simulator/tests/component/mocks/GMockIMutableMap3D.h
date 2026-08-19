#pragma once

#include <Common/IMutableMap3D.h>

#include <gmock/gmock.h>

#include <filesystem>

namespace test {

class GMockIMutableMap3D : public common::IMutableMap3D {
public:
    MOCK_METHOD(common::types::VoxelOccupancy, atVoxel, (const common::Position3D& pos), (const, override));
    MOCK_METHOD(common::types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const common::Position3D& pos), (const, override));
    MOCK_METHOD(void, set, (const common::Position3D& pos, common::types::VoxelOccupancy value), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path& path), (const, override));
};

} // namespace test
