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
#include <string>

namespace mission_control_322889890_315113738 {

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    // Shared completion message for mappings with unmappable voxels.
    static constexpr const char* kUnmappableVoxelsMessage =
        "mapping finished with unmappable voxels remaining";

    DroneControlImpl(common::types::DroneConfigData drone,
                     const common::ILidar& lidar,
                     const common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm,
                     bool verbose = false);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

    // Compact, human-readable description of what happened during the most recently completed
    // step() call (e.g. "Advance 30cm | pos=(180,200,10) | heading=0deg | status=Continue").
    // Not part of IDroneControl -- verbose logging only, queried by MissionControlImpl after
    // each step() call. Observational: built entirely from data step() already gathered, no
    // extra GPS/LiDAR/movement calls.
    [[nodiscard]] const std::string& lastStepLog() const { return last_step_log_; }

private:
    // Stores pending movement chunks and deferred scan/status for one Algorithm command.
    struct PendingMovementSequence {
        std::deque<common::types::MovementCommand> movements;
        std::optional<common::Orientation> scan_orientation;
        common::types::AlgorithmStatus status = common::types::AlgorithmStatus::Working;
        // Heading used to prepare and validate all chunks in this sequence.
        common::Orientation heading;
        // Whether `heading` above still reflects a genuine gps_.heading() reading. True at
        // sequence creation (heading is freshly read then); a dispatched Rotate chunk changes
        // the real heading by an amount this class cannot know without another sensor call, so
        // it clears this rather than guessing -- verbose logging only, never used to affect
        // movement/decisions.
        bool heading_verified = true;
    };

    // Validates the pre-step GPS reading; returns a result only when step() must end early.
    [[nodiscard]] std::optional<common::types::DroneStepResult> handlePreStepGps(
        common::Position3D& gps_position);

    // Validates and prepares one Algorithm command for execution.
    [[nodiscard]] PendingMovementSequence prepareNextSequence(const common::Position3D& gps_position);

    // Returns a valid GPS reading matching the expected post-movement position.
    [[nodiscard]] std::optional<common::Position3D> validatePostMovementGps(
        const common::Position3D& expected);

    // Dispatches one movement chunk and validates the resulting GPS position.
    [[nodiscard]] std::optional<common::types::DroneStepResult> dispatchMovementAndValidateGps(
        const common::types::MovementCommand& movement, const common::Orientation& heading);

    // Performs a LiDAR scan with retries and applies it to the output map.
    [[nodiscard]] std::optional<common::types::DroneStepResult> dispatchScanAndApplyToMap(
        const common::Orientation& scan_orientation, const common::Position3D& post_move_pos,
        const common::Orientation& post_move_heading);

    // Builds and stores the compact verbose-log line for the step() call currently in progress.
    // `heading` and `scan_hits` are omitted from the line when not meaningful for this step.
    void recordStepLog(const std::string& action, const common::Position3D& position,
                       const std::optional<common::Orientation>& heading,
                       std::optional<std::size_t> scan_hits,
                       common::types::DroneStepStatus status, const std::string& reason);

    common::types::DroneConfigData drone_;
    const common::ILidar& lidar_;
    const common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    std::optional<common::types::LidarScanResult> latest_scan_{};
    std::optional<PendingMovementSequence> pending_sequence_;

    // Internal position estimate used only to validate suspicious GPS readings.
    std::optional<common::Position3D> internal_position_;

    // Verbose-log line for the most recently completed step() call; see lastStepLog().
    std::string last_step_log_;

    // When false, step() skips all lastStepLog()-related string building entirely.
    const bool verbose_ = false;
};

} // namespace mission_control_322889890_315113738