#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <utility>

namespace MissionControl_322889890_315113738 {

namespace common_types = common::types;

using common::Orientation;
using common::Position3D;

DroneControlImpl::DroneControlImpl(common_types::DroneConfigData drone,
                                   common_types::MissionConfigData mission,
                                   common::ILidar& lidar,
                                   common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

common_types::DroneStepResult DroneControlImpl::step() {
    const common_types::DroneState state{gps_.position(), gps_.heading(), step_index_};
    const common_types::LidarScanResult* latest_scan_ptr =
        latest_scan_ ? &(*latest_scan_) : nullptr;
    const common_types::MappingStepCommand command =
        mapping_algorithm_.nextStep(state, latest_scan_ptr);

    // Movement is executed before any scan, per MappingStepCommand's contract.
    if (command.movement.has_value()) {
        const common_types::MovementCommand& movement = *command.movement;
        common_types::MovementResult result{};

        switch (movement.type) {
            case common_types::MovementCommandType::Hover:
                break;
            case common_types::MovementCommandType::Rotate:
                result = movement_.rotate(movement.rotation, movement.angle);
                break;
            case common_types::MovementCommandType::Advance:
                result = movement_.advance(movement.distance);
                break;
            case common_types::MovementCommandType::Elevate:
                result = movement_.elevate(movement.distance);
                break;
        }

        if (!result.success) {
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Error, result.message};
        }
    }

    if (command.scan_orientation.has_value()) {
        const Position3D post_move_pos = gps_.position();
        const Orientation post_move_heading = gps_.heading();
        const common_types::LidarScanResult scan =
            lidar_.scan(*command.scan_orientation);

        ScanResultToVoxels::applyToMap(
            output_map_, post_move_pos, post_move_heading, scan, lidar_.config());
        latest_scan_ = scan;
    } else {
        latest_scan_ = std::nullopt;
    }

    ++step_index_;

    switch (command.status) {
        case common_types::AlgorithmStatus::Working:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Continue, "working"};

        case common_types::AlgorithmStatus::Finished:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Completed, "mapping finished"};

        case common_types::AlgorithmStatus::FinishedWithUnmappableVoxels:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Completed,
                "mapping finished with unmappable voxels remaining"};
    }

    return common_types::DroneStepResult{
        common_types::DroneStepStatus::Error,
        "DroneControlImpl::step: unhandled AlgorithmStatus."};
}

common_types::DroneState DroneControlImpl::state() const {
    return common_types::DroneState{
        gps_.position(), gps_.heading(), step_index_};
}

} // namespace MissionControl_322889890_315113738