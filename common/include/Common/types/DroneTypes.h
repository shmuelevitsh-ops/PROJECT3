#pragma once

#include <Common/Units.h>

#include <cstddef>
#include <optional>
#include <string>

namespace common::types {

struct DroneConfigData {
    PhysicalLength radius{};
    HorizontalAngle max_rotate{};
    PhysicalLength max_advance{};
    PhysicalLength max_elevate{};
};

enum class RotationDirection { Left, Right };
enum class MovementCommandType { Hover, Rotate, Advance, Elevate };

struct MovementCommand {
    MovementCommandType type = MovementCommandType::Hover;
    RotationDirection rotation = RotationDirection::Left;
    HorizontalAngle angle{};
    PhysicalLength distance{};
};

enum class AlgorithmStatus { Working, Finished, FinishedWithUnmappableVoxels };

struct MappingStepCommand {
    std::optional<MovementCommand> movement{};
    std::optional<Orientation> scan_orientation{};
    AlgorithmStatus status = AlgorithmStatus::Working;
};

struct MovementResult {
    bool success = true;
    std::string message{};

    [[nodiscard]] explicit operator bool() const noexcept { return success; }
};

struct DroneState {
    Position3D position{};
    Orientation heading{};
    std::size_t step_index = 0;
};

enum class DroneStepStatus { Continue, Completed, Error };

struct DroneStepResult {
    DroneStepStatus status = DroneStepStatus::Continue;
    std::string message{};
};

} // namespace common::types
