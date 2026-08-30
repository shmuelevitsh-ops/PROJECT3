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

// Places the verbose log next to the run's output map.
[[nodiscard]] std::filesystem::path verboseLogPath(std::filesystem::path output_map_file) {
    output_map_file.replace_extension();
    output_map_file += "_verbose.log";
    return output_map_file;
}

// Opens and initializes the verbose log when enabled.
[[nodiscard]] std::ofstream openVerboseLog(bool verbose, const std::filesystem::path& output_map_file,
                                           std::size_t max_steps) {
    std::ofstream verbose_log;
    if (!verbose) {
        return verbose_log;
    }
    const std::filesystem::path verbose_log_file = verboseLogPath(output_map_file);
    verbose_log.open(verbose_log_file);
    if (verbose_log.is_open()) {
        verbose_log << "mission started: max_steps=" << max_steps << '\n';
    } else {
        std::cerr << "MissionControlImpl::runMission: failed to open verbose log file: "
                << verbose_log_file << '\n';
    }
    return verbose_log;
}

// Records map-save failures without changing the mission outcome.
void saveOutputMap(const common::IMutableMap3D& output_map, const std::filesystem::path& output_map_file,
                   std::vector<common_types::ErrorRef>& errors, std::ofstream& verbose_log) {
    try {
        output_map.save(output_map_file);
    } catch (const std::exception& e) {
        std::cerr << "MissionControlImpl::runMission: failed to save output map: " << e.what() << '\n';
        errors.push_back(common_types::ErrorRef{"OUTPUT_MAP_SAVE_FAILED", e.what()});
        if (verbose_log.is_open()) {
            verbose_log << "output map save failed: " << e.what() << '\n';
        }
    }
}

// Writes the final mission-finished summary line, iff `verbose_log` is open.
void writeVerboseSummary(std::ofstream& verbose_log, common_types::MissionRunStatus status,
                         std::size_t steps, std::size_t error_count,
                         const std::filesystem::path& output_map_file,
                         const std::string& completion_message) {
    if (!verbose_log.is_open()) {
        return;
    }
    verbose_log << "mission finished: status=" << toString(status) << " steps=" << steps
                << " errors=" << error_count << " output_map=" << output_map_file.string();
    if (!completion_message.empty()) {
        verbose_log << " completion_reason=" << completion_message;
    }
    verbose_log << '\n';
}

} // namespace

MissionControlImpl::MissionControlImpl(common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      drone_control_(std::make_unique<DroneControlImpl>(dependencies.drone_config,
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
    // Records map-save failures without changing the mission outcome.
    std::string completion_message;

    // Opened only when -verbose is set; every write below is guarded by is_open(), so this stays
    // a no-op (no file created, nothing written) otherwise.
    std::ofstream verbose_log = openVerboseLog(verbose_, output_map_file_, mission_.max_steps);

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
            if (result.message == DroneControlImpl::kUnmappableVoxelsMessage) {
                std::cerr << "MissionControlImpl::runMission: " << result.message << '\n';
                errors.push_back(common_types::ErrorRef{"UNMAPPABLE_VOXELS_REMAINING", result.message});
                if (verbose_log.is_open()) {
                    verbose_log << "step " << steps << ": " << result.message << '\n';
                }
            }
            break;
        }

        // DroneStepStatus::Error is recorded but does not terminate the mission.
        std::cerr << "MissionControlImpl::runMission: drone control error: " << result.message << '\n';
        errors.push_back(common_types::ErrorRef{"DRONE_CONTROL_ERROR", result.message});
        if (verbose_log.is_open()) {
            verbose_log << "step " << steps << ": drone control error: " << result.message << '\n';
        }
    }

    saveOutputMap(output_map_, output_map_file_, errors, verbose_log);

    writeVerboseSummary(verbose_log, status, steps, errors.size(), output_map_file_, completion_message);

    return common_types::MissionRunResult{status, steps, errors};
}

REGISTER_MISSION_CONTROL(MissionControlImpl);

} // namespace mission_control_322889890_315113738
