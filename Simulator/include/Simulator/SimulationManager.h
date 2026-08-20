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

    // ISimulation overload: generates synthetic config names because
    // the interface does not provide the original config file paths.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

    // Simulator flow overload: uses the real config paths and load errors
    // returned by ConfigLoader.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path,
        const CompositionFilePaths& file_paths);

private:
    // Shared implementation used by both public run() overloads.
    [[nodiscard]] types::SimulationManagerReport runInternal(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path,
        const CompositionFilePaths& file_paths);

    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace simulator