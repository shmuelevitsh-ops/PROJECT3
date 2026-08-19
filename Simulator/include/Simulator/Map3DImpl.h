#pragma once

#include <TinyNPY.h>

#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace simulator {

class Map3DImpl final : public common::IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    Map3DImpl(
        std::shared_ptr<NpyArray> map_ptr,
        const common::types::MapConfig map_config);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(
        const common::Position3D& pos) const override;

    [[nodiscard]] common::types::MapConfig getMapConfig() const override;

    [[nodiscard]] bool isInBounds(
        const common::Position3D& pos) const override;

    void set(
        const common::Position3D& pos,
        common::types::VoxelOccupancy value) override;

    void save(const std::filesystem::path& output_path) const override;

private:
    std::shared_ptr<NpyArray> map_;
    common::types::MapConfig config_;
};

} // namespace simulator
