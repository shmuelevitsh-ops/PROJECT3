#pragma once

#include <Simulator/ISimulationRunFactory.h>

#include <gmock/gmock.h>

namespace test {

class GMockISimulationRunFactory : public simulator::ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<simulator::ISimulationRun>, create,
                (const simulator::types::SimulationConfigData& simulation,
                 const common::types::MissionConfigData& mission, const common::types::DroneConfigData& drone,
                 const common::types::LidarConfigData& lidar, const std::filesystem::path& output_path),
                (override));
};

} // namespace test
