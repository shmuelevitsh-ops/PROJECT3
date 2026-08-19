#pragma once

#include <Simulator/SimulationTypes.h>

namespace simulator {

class ISimulation {
public:
    virtual ~ISimulation() = default;
    [[nodiscard]] virtual types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) = 0;
};

} // namespace simulator
