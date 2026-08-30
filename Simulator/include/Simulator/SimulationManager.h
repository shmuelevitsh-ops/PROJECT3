#pragma once

#include <Simulator/ConfigLoader.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>

namespace simulator {

class SimulationManager final : public ISimulation {
public:
    // file_paths must match the simulation/mission/drone/lidar structure passed to run().
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, CompositionFilePaths file_paths);

    // file_paths must match the simulation/mission/drone/lidar structure passed to run().
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    CompositionFilePaths file_paths_;
};

} // namespace simulator
