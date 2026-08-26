#include <Simulator/SimulationManager.h>

#include <Simulator/CerrContextGuard.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace simulator {

namespace common_types = common::types;

using common::Position3D;

namespace {

constexpr const char* kMetric = "output_map_accuracy";
constexpr double kScoreRangeMin = 0.0;
constexpr double kScoreRangeMax = 100.0;
constexpr int kErrorScore = -1;

// Returns the current UTC time in ISO-like format for the final report.
std::string currentUtcTimestamp() {
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc_tm{};
    gmtime_r(&now_time_t, &utc_tm);
    std::ostringstream out;
    out << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

std::string stem(const std::string& path_str) {
    return std::filesystem::path(path_str).stem().string();
}

// Builds a unique output directory for one simulation run.
// If the same directory name was already used, appends "__2", "__3", etc.
std::filesystem::path uniqueLeafDir(const std::filesystem::path& output_path, const std::string& sim_stem,
                                    const std::string& mission_stem, const std::string& drone_stem,
                                    const std::string& lidar_stem, std::set<std::filesystem::path>& used_leaf_dirs) {
    const std::filesystem::path base =
        output_path / "simulations" / sim_stem / mission_stem / (drone_stem + "__" + lidar_stem);

    std::filesystem::path candidate = base;
    for (int suffix = 2; used_leaf_dirs.find(candidate) != used_leaf_dirs.end(); ++suffix) {
        candidate = base;
        candidate += "__" + std::to_string(suffix);
    }
    used_leaf_dirs.insert(candidate);
    return candidate;
}

// Builds a readable run identifier for contextual error logging.
std::string contextLabel(const std::string& sim_stem, const std::string& mission_stem, const std::string& drone_stem,
                         const std::string& lidar_stem) {
    return "sim=" + sim_stem + " mission=" + mission_stem + " drone=" + drone_stem + " lidar=" + lidar_stem;
}

// Builds a failed SimulationResult so one bad run can be scored -1
// without aborting the rest of the composition.
// Preserves available simulation/mission metadata for the final report.
types::SimulationResult buildErrorResult(const types::SimulationConfigData& simulation,
                                        const common_types::MissionConfigData& mission,
                                        common_types::ErrorRef error) {
    types::SimulationResult result;
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.output_map_config.boundaries = mission.mission_bounds;
    // map_local = mission_relative + offset, so voxel index 0 sits at mission_relative = -offset;
    // mirrors SimulationRunFactoryImpl::outputMapConfig()'s offset for this same output map.
    result.output_map_config.offset =
        Position3D{-mission.mission_bounds.min_x, -mission.mission_bounds.min_y, -mission.mission_bounds.min_height};
    result.output_map_config.resolution = outputMapResolution(simulation);
    result.mission_score = kErrorScore;

    common_types::MissionRunResult mission_result;
    mission_result.status = common_types::MissionRunStatus::Error;
    mission_result.steps = 0;
    mission_result.errors.push_back(std::move(error));
    result.mission_results = {mission_result};
    return result;
}

// Converts a caught exception into an ErrorRef.
// Preserves the specific error code for SimulationException.
types::SimulationResult buildErrorResult(const types::SimulationConfigData& simulation,
                                        const common_types::MissionConfigData& mission,
                                        const std::exception& e) {
    if (const auto* simulation_exception = dynamic_cast<const SimulationException*>(&e)) {
        return buildErrorResult(simulation, mission,
                                common_types::ErrorRef{simulation_exception->code(), simulation_exception->what()});
    }
    return buildErrorResult(simulation, mission, common_types::ErrorRef{"RUN_INITIALIZATION_ERROR", e.what()});
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                                     CompositionFilePaths file_paths)
    : run_factory_(std::move(run_factory)), file_paths_(std::move(file_paths)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

// Executes every simulation × mission × drone × lidar combination.
// Converts run-level failures into -1 results and aggregates all runs into the final report.
types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    std::set<std::filesystem::path> used_leaf_dirs;
    // Create one run for every simulation × mission × drone × lidar combination.
    for (std::size_t sim_index = 0; sim_index < composition.simulation_mission_groups.size(); ++sim_index) {
        const auto& [simulation, missions] = composition.simulation_mission_groups[sim_index];
        const auto& [sim_ref, mission_refs] = file_paths_.simulation_mission_paths[sim_index];
        const std::string sim_stem = stem(sim_ref.path);

        for (std::size_t mission_index = 0; mission_index < missions.size(); ++mission_index) {
            const common_types::MissionConfigData& mission = missions[mission_index];
            const ReferencedConfigFile& mission_ref = mission_refs[mission_index];
            const std::string mission_stem = stem(mission_ref.path);

            for (std::size_t drone_index = 0; drone_index < composition.drone_configs.size(); ++drone_index) {
                const common_types::DroneConfigData& drone = composition.drone_configs[drone_index];
                const std::string drone_stem = stem(file_paths_.drone_paths[drone_index]);


                for (std::size_t lidar_index = 0; lidar_index < composition.lidar_configs.size(); ++lidar_index) {
                    const common_types::LidarConfigData& lidar = composition.lidar_configs[lidar_index];
                    const std::string lidar_stem = stem(file_paths_.lidar_paths[lidar_index]);

                    const std::filesystem::path leaf_dir =
                        uniqueLeafDir(output_path, sim_stem, mission_stem, drone_stem, lidar_stem, used_leaf_dirs);
                    std::filesystem::create_directories(leaf_dir);
                    // Attach this run's identity to any cerr output produced in this scope.
                    const CerrContextGuard cerr_guard(contextLabel(sim_stem, mission_stem, drone_stem, lidar_stem));
                    // Skip factory creation when ConfigLoader already marked this config as invalid.
                    if (sim_ref.load_error) {
                        std::cerr << "SimulationManager::run: simulation_config failed to load, scoring -1: "
                            << sim_ref.load_error->message << '\n';

                        runs.push_back(buildErrorResult(simulation, mission, *sim_ref.load_error));

                    } else if (mission_ref.load_error) {
                        std::cerr << "SimulationManager::run: mission_config failed to load, scoring -1: "
                            << mission_ref.load_error->message << '\n';

                        runs.push_back(buildErrorResult(simulation, mission, *mission_ref.load_error));
                    } else {
                        // Build and execute this run; convert any per-run failure into a -1 result.
                        try {
                            std::unique_ptr<ISimulationRun> run =
                                run_factory_->create(simulation, mission, drone, lidar, leaf_dir);
                            runs.push_back(run->run());
                        } catch (const std::exception& e) {
                            std::cerr << "SimulationManager::run: run failed, scoring -1: " << e.what() << '\n';
                            runs.push_back(buildErrorResult(simulation, mission, e));
                        }
                    }
                }
            }
        }
    }

    return types::SimulationManagerReport{composition.composition_file, currentUtcTimestamp(),
                                        kMetric, {kScoreRangeMin, kScoreRangeMax}, kErrorScore,
                                        std::move(runs)};
}

} // namespace simulator
