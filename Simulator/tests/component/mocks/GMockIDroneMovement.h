#pragma once

#include <Common/IDroneMovement.h>

#include <gmock/gmock.h>

namespace test {

class GMockIDroneMovement : public common::IDroneMovement {
public:
    MOCK_METHOD(common::types::MovementResult, rotate,
                (common::types::RotationDirection direction, common::HorizontalAngle angle), (override));
    MOCK_METHOD(common::types::MovementResult, advance, (common::PhysicalLength distance), (override));
    MOCK_METHOD(common::types::MovementResult, elevate, (common::PhysicalLength distance), (override));
};

} // namespace test
