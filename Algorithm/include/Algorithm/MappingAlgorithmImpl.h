#pragma once

#include <Common/IMappingAlgorithm.h>

#include <memory>

namespace algorithm_322889890_315113738 {

class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    // Receives the algorithm's configuration and output map dependencies.
    explicit MappingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~MappingAlgorithmImpl() override;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace algorithm_322889890_315113738