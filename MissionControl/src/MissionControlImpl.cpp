#include <MissionControl/MissionControlImpl.h>

#include <MissionControl/DroneControlImpl.h>
#include <Common/MissionControlRegistration.h>

#include <iostream>
#include <utility>

namespace MissionControl_322889890_315113738 {

namespace common_types = common::types;

namespace {

constexpr const char* kUnmappableVoxelsMessage = "mapping finished with unmappable voxels remaining";

} // namespace

MissionControlImpl::MissionControlImpl(common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      drone_control_(std::make_unique<DroneControlImpl>(dependencies.drone_config,
                                                          dependencies.mission_config,
                                                          dependencies.lidar,
                                                          dependencies.gps,
                                                          dependencies.movement,
                                                          dependencies.output_map,
                                                          dependencies.mapping_algorithm)),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose) {}

common_types::MissionRunResult MissionControlImpl::runMission() {
    std::vector<common_types::ErrorRef> errors;
    common_types::MissionRunStatus status = common_types::MissionRunStatus::MaxSteps;
    std::size_t steps = 0;

    while (steps < mission_.max_steps) {
        const common_types::DroneStepResult result = drone_control_->step();
        ++steps;

        if (result.status == common_types::DroneStepStatus::Continue) {
            continue;
        }

        if (result.status == common_types::DroneStepStatus::Completed) {
            status = common_types::MissionRunStatus::Completed;
            if (result.message == kUnmappableVoxelsMessage) {
                std::cerr << "MissionControlImpl::runMission: " << result.message << '\n';
                errors.push_back(common_types::ErrorRef{"UNMAPPABLE_VOXELS_REMAINING", result.message});
            }
            break;
        }

        // DroneStepStatus::Error — non-terminal at MissionControl level: log it, record it, and
        // keep going. The mission only ends in Error via other paths (e.g. an unhandled
        // exception); a returned Error alone must still resolve to Completed or MaxSteps.
        std::cerr << "MissionControlImpl::runMission: drone control error: " << result.message << '\n';
        errors.push_back(common_types::ErrorRef{"DRONE_CONTROL_ERROR", result.message});
    }

    output_map_.save(output_map_file_);
    return common_types::MissionRunResult{status, steps, errors};
}

REGISTER_MISSION_CONTROL(MissionControlImpl);

} // namespace MissionControl_322889890_315113738
