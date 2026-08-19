#pragma once

#include <Simulator/ISimulationRun.h>

#include <memory>

namespace simulator {

class ISimulationRunFactory {
public:
    virtual ~ISimulationRunFactory() = default;
    [[nodiscard]] virtual std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& drone_config,
        const common::types::LidarConfigData& lidar_config,
        const std::filesystem::path& output_path) = 0;
};

} // namespace simulator
