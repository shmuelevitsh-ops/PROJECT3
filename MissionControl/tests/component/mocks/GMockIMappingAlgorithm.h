#pragma once

#include <Common/IMappingAlgorithm.h>

#include <gmock/gmock.h>

#include <utility>

namespace test {

class GMockIMappingAlgorithm : public common::IMappingAlgorithm {
public:
    explicit GMockIMappingAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    MOCK_METHOD(common::types::MappingStepCommand, nextStep,
                (const common::types::DroneState& state, const common::types::LidarScanResult* latest_scan),
                (override));
};

} // namespace test
