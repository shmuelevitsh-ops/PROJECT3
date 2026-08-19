#pragma once

#include <Simulator/ISimulationRun.h>

#include <gmock/gmock.h>

namespace test {

class GMockISimulationRun : public simulator::ISimulationRun {
public:
    MOCK_METHOD(simulator::types::SimulationResult, run, (), (override));
};

} // namespace test
