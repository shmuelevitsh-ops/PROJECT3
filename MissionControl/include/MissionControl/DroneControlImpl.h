#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <MissionControl/IDroneControl.h>

#include <cstddef>
#include <deque>
#include <optional>

namespace MissionControl_322889890_315113738 {

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    // A single Algorithm-issued MappingStepCommand, after the existing OOB amendment has been
    // applied and the resulting legal movement (if any) has been split into DroneConfigData-sized
    // chunks (Optional Common-Issues: "Algorithm returned a movement bigger than the max
    // allowed"). `movements` holds the chunks not yet dispatched (front() is next); the original
    // command's scan_orientation/status are deferred here and only applied once `movements` is
    // empty after a dispatch, i.e. on the final chunk.
    struct PendingMovementSequence {
        std::deque<common::types::MovementCommand> movements;
        std::optional<common::Orientation> scan_orientation;
        common::types::AlgorithmStatus status = common::types::AlgorithmStatus::Working;
        // Heading in effect when this sequence's movement was legalized/split, i.e. the heading
        // used by every chunk of it (Rotate is never split alongside Advance/Elevate, and
        // Advance/Elevate chunks never rotate mid-sequence, so one heading covers the whole
        // sequence). Captured once here, from the same GPS read prepareNextSequence() already
        // performs, rather than re-reading gps_.heading() once per chunk purely to recompute the
        // expected post-movement position (Optional Common-Issues row 12).
        common::Orientation heading;
    };

    // Fetches and prepares the next MappingStepCommand from mapping_algorithm_ into a
    // PendingMovementSequence. Called only when pending_sequence_ is empty, i.e. once per
    // Algorithm-issued command, never once per chunk. `gps_position` is the GPS reading step()
    // already took at the top of this call (Optional Common-Issues row 11 validation) --
    // reused here instead of re-reading gps_.position() a second time.
    [[nodiscard]] PendingMovementSequence prepareNextSequence(const common::Position3D& gps_position);

    // Reads gps_.position() (and up to 3 additional retries) looking for a reading that is both
    // in bounds and matches `expected` within numerical tolerance (Optional Common-Issues row 12:
    // "A movement executed but GPS updated with impossible coordinates"). Returns the first
    // reading meeting both conditions, or std::nullopt if every attempt fails either one.
    [[nodiscard]] std::optional<common::Position3D> validatePostMovementGps(
        const common::Position3D& expected);

    common::types::DroneConfigData drone_;
    common::types::MissionConfigData mission_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    std::optional<common::types::LidarScanResult> latest_scan_{};
    std::optional<PendingMovementSequence> pending_sequence_;

    // Self-maintained plausibility/reference estimate of the drone's position (Optional
    // Common-Issues rows 11/12) -- NOT a replacement source of truth for GPS. Initialized from
    // the first in-bounds GPS reading ever seen; thereafter it evolves only from successfully
    // executed movement (see prepareNextSequence/step), optionally resynchronized to a GPS
    // reading that validates against that expectation. Never substituted into Algorithm state or
    // scan-map writes in place of a GPS sample that was itself rejected.
    std::optional<common::Position3D> internal_position_;
};

} // namespace MissionControl_322889890_315113738