#pragma once

#include <Simulator/SimulationTypes.h>

namespace simulator {

class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;
    [[nodiscard]] virtual types::SimulationResult run() = 0;
};

} // namespace simulator
