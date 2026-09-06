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
    struct RunAccumulator {
        std::vector<types::SimulationResult> runs;
        std::size_t construction_attempts = 0;
        std::size_t construction_successes = 0;
    };

    // Handles one simulation × mission × drone × lidar combination: creates its output
    // directory, skips construction if the simulation/mission config failed to load, and
    // otherwise delegates to constructAndRunScenario(). Appends exactly one result to
    // accumulator.runs.
    void runScenario(const types::SimulationConfigData& simulation, const common::types::MissionConfigData& mission,
                     const common::types::DroneConfigData& drone, const common::types::LidarConfigData& lidar,
                     const ReferencedConfigFile& sim_ref, const ReferencedConfigFile& mission_ref,
                     const std::filesystem::path& leaf_dir, RunAccumulator& accumulator);

    // Constructs this scenario's run via run_factory_ and executes it, converting any
    // per-run failure (including a failed component construction) into a -1 result.
    void constructAndRunScenario(const types::SimulationConfigData& simulation,
                                 const common::types::MissionConfigData& mission,
                                 const common::types::DroneConfigData& drone,
                                 const common::types::LidarConfigData& lidar,
                                 const std::filesystem::path& leaf_dir, RunAccumulator& accumulator);

    // Handles a ComponentConstructionException thrown by run_factory_->create(): scores -1 and
    // updates accumulator.construction_attempts/construction_successes per the rules described
    // in run()'s banner comment (in SimulationManager.cpp).
    void handleComponentConstructionFailure(const ComponentConstructionException& e,
                                            const types::SimulationConfigData& simulation,
                                            const common::types::MissionConfigData& mission,
                                            RunAccumulator& accumulator);

    std::unique_ptr<ISimulationRunFactory> run_factory_;
    CompositionFilePaths file_paths_;
    ComponentKind evaluated_component_;
};

} // namespace simulator
