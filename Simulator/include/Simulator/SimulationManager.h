#pragma once

#include <Simulator/ConfigLoader.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>

namespace simulator {

class SimulationManager final : public ISimulation {
public:
    // file_paths must describe the same simulation/mission/drone/lidar shape as every
    // composition later passed to run(): one simulation_mission_paths entry (with one nested
    // mission entry per mission) per simulation_mission_groups entry, one drone_paths entry per
    // drone_configs entry, one lidar_paths entry per lidar_configs entry.
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, CompositionFilePaths file_paths);

    // Names each run's output directory from the stored file_paths' real config file stems, and
    // uses their load_error entries to recognize a config ConfigLoader already marked as failed to
    // load, before ever calling the run factory for it.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    CompositionFilePaths file_paths_;
};

} // namespace simulator
