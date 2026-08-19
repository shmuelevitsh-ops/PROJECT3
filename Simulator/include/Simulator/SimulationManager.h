#pragma once

#include <Simulator/ConfigLoader.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>

namespace simulator {

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(
        std::unique_ptr<ISimulationRunFactory> run_factory);

    // Required ISimulation interface.
    // Synthetic path names are generated because this overload does not
    // receive the original configuration-file paths.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

    // Simulator-specific overload used after parseCompositionData().
    // Receives the real configuration paths and their optional load errors.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path,
        const CompositionFilePaths& file_paths);

private:
    [[nodiscard]] types::SimulationManagerReport runInternal(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path,
        const CompositionFilePaths& file_paths);

    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace simulator