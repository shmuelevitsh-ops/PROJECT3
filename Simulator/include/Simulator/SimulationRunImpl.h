#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace simulator {

class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<const common::IMap3D> hidden_map,
                    std::unique_ptr<common::IMutableMap3D> output_map,
                    std::unique_ptr<common::IGPS> gps,
                    std::unique_ptr<common::IDroneMovement> movement,
                    std::unique_ptr<common::ILidar> lidar,
                    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                    std::unique_ptr<common::IMissionControl> mission_control,
                    types::SimulationConfigData simulation_config,
                    common::types::MissionConfigData mission_config,
                    std::filesystem::path output_map_file);

    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const common::IMap3D> hidden_map_;
    std::unique_ptr<common::IMutableMap3D> output_map_;
    std::unique_ptr<common::IGPS> gps_;
    std::unique_ptr<common::IDroneMovement> movement_;
    std::unique_ptr<common::ILidar> lidar_;
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<common::IMissionControl> mission_control_;

    types::SimulationConfigData simulation_config_;
    common::types::MissionConfigData mission_config_;
    std::filesystem::path output_map_file_;
};

} // namespace simulator