#include <MissionControl/MissionControlImpl.h>

#include <MissionControl/DroneControlImpl.h>
#include <Common/MissionControlRegistration.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <utility>

namespace mission_control_322889890_315113738 {

namespace common_types = common::types;

namespace {

constexpr const char* kUnmappableVoxelsMessage = "mapping finished with unmappable voxels remaining";

// Verbose logging is event-based, not per-step (missions can run for many thousands of steps):
// a lightweight checkpoint line is written only every kVerboseCheckpointInterval completed steps.
constexpr std::size_t kVerboseCheckpointInterval = 500;

[[nodiscard]] const char* toString(common_types::MissionRunStatus status) {
    switch (status) {
        case common_types::MissionRunStatus::Completed: return "Completed";
        case common_types::MissionRunStatus::MaxSteps: return "MaxSteps";
        case common_types::MissionRunStatus::Error: return "Error";
    }
    return "Unknown";
}

// Verbose diagnostics live next to the run's own output map, so the two files are always found
// together in the same per-mission output directory -- e.g. ".../map_output.npy" ->
// ".../map_output_verbose.log".
[[nodiscard]] std::filesystem::path verboseLogPath(std::filesystem::path output_map_file) {
    output_map_file.replace_extension();
    output_map_file += "_verbose.log";
    return output_map_file;
}

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
    // The message naturally attached to whichever event actually ended the mission (a terminal
    // DroneStepResult::message, or an exception's what()) -- left empty for a plain MaxSteps
    // ending, where no such message exists.
    std::string completion_message;

    // Opened only when -verbose is set; every write below is guarded by is_open(), so this stays
    // a no-op (no file created, nothing written) otherwise.
    std::ofstream verbose_log;
    if (verbose_) {
        const std::filesystem::path verbose_log_file = verboseLogPath(output_map_file_);
        verbose_log.open(verbose_log_file);
        if (verbose_log.is_open()) {
            verbose_log << "mission started: max_steps=" << mission_.max_steps << '\n';
        } else {
            std::cerr << "MissionControlImpl::runMission: failed to open verbose log file: "
                    << verbose_log_file << '\n';
        }
    }

    while (steps < mission_.max_steps) {
        common_types::DroneStepResult result;

        try {
            result = drone_control_->step();
        } catch (const std::exception& e) {
            std::cerr << "MissionControlImpl::runMission: drone control exception: "
                    << e.what() << '\n';

            errors.push_back(
                common_types::ErrorRef{"DRONE_CONTROL_EXCEPTION", e.what()});

            status = common_types::MissionRunStatus::Error;
            completion_message = e.what();
            if (verbose_log.is_open()) {
                verbose_log << "step " << (steps + 1) << ": drone control exception: "
                            << e.what() << '\n';
            }
            break;
        }

        ++steps;

        if (verbose_log.is_open() && steps % kVerboseCheckpointInterval == 0) {
            verbose_log << "checkpoint: steps=" << steps << '\n';
        }

        if (result.status == common_types::DroneStepStatus::Continue) {
            continue;
        }

        if (result.status == common_types::DroneStepStatus::Completed) {
            status = common_types::MissionRunStatus::Completed;
            completion_message = result.message;
            if (result.message == kUnmappableVoxelsMessage) {
                std::cerr << "MissionControlImpl::runMission: " << result.message << '\n';
                errors.push_back(common_types::ErrorRef{"UNMAPPABLE_VOXELS_REMAINING", result.message});
                if (verbose_log.is_open()) {
                    verbose_log << "step " << steps << ": " << result.message << '\n';
                }
            }
            break;
        }

        // DroneStepStatus::Error — non-terminal at MissionControl level: log it, record it, and
        // keep going. The mission only ends in Error via other paths (e.g. an unhandled
        // exception); a returned Error alone must still resolve to Completed or MaxSteps.
        std::cerr << "MissionControlImpl::runMission: drone control error: " << result.message << '\n';
        errors.push_back(common_types::ErrorRef{"DRONE_CONTROL_ERROR", result.message});
        if (verbose_log.is_open()) {
            verbose_log << "step " << steps << ": drone control error: " << result.message << '\n';
        }
    }

    // The mission's outcome (status/steps/errors) is already fully determined at this point;
    // output_map_.save() failing must not erase it by propagating out of runMission() -- it is
    // reported as an additional error instead, leaving status/steps/errors otherwise untouched.
    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        std::cerr << "MissionControlImpl::runMission: failed to save output map: " << e.what() << '\n';
        errors.push_back(common_types::ErrorRef{"OUTPUT_MAP_SAVE_FAILED", e.what()});
        if (verbose_log.is_open()) {
            verbose_log << "output map save failed: " << e.what() << '\n';
        }
    }

    if (verbose_log.is_open()) {
        verbose_log << "mission finished: status=" << toString(status) << " steps=" << steps
                    << " errors=" << errors.size() << " output_map=" << output_map_file_.string();
        if (!completion_message.empty()) {
            verbose_log << " completion_reason=" << completion_message;
        }
        verbose_log << '\n';
    }

    return common_types::MissionRunResult{status, steps, errors};
}

REGISTER_MISSION_CONTROL(MissionControlImpl);

} // namespace mission_control_322889890_315113738
