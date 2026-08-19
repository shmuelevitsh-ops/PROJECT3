#pragma once

#include <Common/Types.h>

namespace common {

class IDroneMovement {
public:
    virtual ~IDroneMovement() = default;
    virtual types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) = 0;
    virtual types::MovementResult advance(PhysicalLength distance) = 0;
    virtual types::MovementResult elevate(PhysicalLength distance) = 0;
};

} // namespace common
