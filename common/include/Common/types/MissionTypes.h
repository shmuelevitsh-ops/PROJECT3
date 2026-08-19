#pragma once

#include <Common/types/MapTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace common::types {

struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 0.0;
    MappingBounds mission_bounds{};
};

enum class MissionRunStatus { Completed, MaxSteps, Error };

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    std::vector<ErrorRef> errors{};
};

} // namespace common::types
