#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

// Applies movement against the hidden map and prevents collisions with Occupied voxels.
class MockMovement final : public common::IDroneMovement {
public:
    MockMovement(MockGPS& gps, const common::IMap3D& hidden_map, common::PhysicalLength drone_radius);

    common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        common::HorizontalAngle angle) override;

    // Throws MOVEMENT_COLLISION if the drone's safety sphere hits an Occupied voxel.
    common::types::MovementResult advance(
        common::PhysicalLength distance) override;

    // Same collision contract as advance(), along the vertical path.
    common::types::MovementResult elevate(
        common::PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const common::IMap3D& hidden_map_;
    common::PhysicalLength drone_radius_;
};

} // namespace simulator
