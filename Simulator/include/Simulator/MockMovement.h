// TODO(A3 collision detection):
// Give MockMovement access to the hidden map so it can reject movements
// that collide with occupied voxels in the real map.

#pragma once

#include <Common/IDroneMovement.h>
#include <Simulator/MockGPS.h>

namespace simulator {
class MockMovement final : public common::IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);

    common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        common::HorizontalAngle angle) override;

    common::types::MovementResult advance(
        common::PhysicalLength distance) override;

    common::types::MovementResult elevate(
        common::PhysicalLength distance) override;

private:
    MockGPS& gps_;
};

} // namespace simulator
