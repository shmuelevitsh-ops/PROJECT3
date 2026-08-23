#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

// Movement driver for the real (hidden) map: as a mock, it is the one component allowed to see
// the hidden map, so it independently rejects any Advance/Elevate whose path would collide with
// a real Occupied wall -- even if the algorithm driving it (potentially another team's faulty
// Algorithm.so) never checks for that itself.
class MockMovement final : public common::IDroneMovement {
public:
    MockMovement(MockGPS& gps, const common::IMap3D& hidden_map, common::PhysicalLength drone_radius);

    common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        common::HorizontalAngle angle) override;

    // Throws simulator::SimulationException ("MOVEMENT_COLLISION") without moving the drone if
    // the drone's safety sphere would intersect an Occupied voxel anywhere along the path,
    // including the exact destination.
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
