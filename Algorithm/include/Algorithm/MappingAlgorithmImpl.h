#pragma once

#include <Common/IMappingAlgorithm.h>

#include <memory>

namespace Algorithm_322889890_315113738 {

class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    // MappingAlgorithmDependencies contains the mission, lidar and drone
    // configurations, together with the output map.
    explicit MappingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~MappingAlgorithmImpl() override;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Algorithm_322889890_315113738