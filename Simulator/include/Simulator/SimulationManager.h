#pragma once

#include <Simulator/ConfigLoader.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationException.h>

#include <filesystem>
#include <memory>

namespace simulator {

class SimulationManager final : public ISimulation {
public:
    // file_paths must match the simulation/mission/drone/lidar structure passed to run().
    // evaluated_component says which injected factory (MissionControl in comparative mode,
    // MappingAlgorithm in competition mode) this SimulationManager is evaluating -- a
    // ComponentConstructionException tagged with the OTHER kind (the fixed/shared component) is
    // treated as an ordinary per-run failure and never counts toward classifying this component as
    // failed (see SimulationManager.cpp). The default is irrelevant to any caller whose run_factory
    // never throws ComponentConstructionException.
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, CompositionFilePaths file_paths,
                      ComponentKind evaluated_component = ComponentKind::MissionControl);

    // file_paths must match the simulation/mission/drone/lidar structure passed to run().
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    CompositionFilePaths file_paths_;
    ComponentKind evaluated_component_;
};

} // namespace simulator
