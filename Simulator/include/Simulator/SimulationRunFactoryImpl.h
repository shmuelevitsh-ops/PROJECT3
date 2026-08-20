#pragma once

#include <Simulator/ISimulationRunFactory.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <memory>

namespace simulator {

// The output map uses the hidden map's resolution from the simulation config.
// SimulationManager also uses this value when reporting a run that failed before the map was built.
[[nodiscard]] common::PhysicalLength outputMapResolution(
    const types::SimulationConfigData& simulation);

// Builds all runtime dependencies for one simulation/mission/drone/lidar combination.
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(common::MappingAlgorithmFactory mapping_algorithm_factory,
                              common::MissionControlFactory mission_control_factory,
                              bool verbose);

    [[nodiscard]] std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation,
        const common::types::MissionConfigData& mission,
        const common::types::DroneConfigData& drone,
        const common::types::LidarConfigData& lidar,
        const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory mapping_algorithm_factory_;
    common::MissionControlFactory mission_control_factory_;
    bool verbose_;
};

} // namespace simulator
