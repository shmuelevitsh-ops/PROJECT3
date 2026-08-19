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
#include <optional>

namespace simulator {

namespace common_types = common::types;

using common::Position3D;

namespace {

constexpr const char* kMetric = "output_map_accuracy";
constexpr double kScoreRangeMin = 0.0;
constexpr double kScoreRangeMax = 100.0;
constexpr int kErrorScore = -1;

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

// Builds a CompositionFilePaths with the same nested shape as `composition`, using positional
// placeholder names ("sim_0", "mission_0_1", "drone_0", "lidar_1", ...) instead of real file
// stems. Used by the 2-arg run() overload, whose callers (e.g. ISimulation-interface callers,
// or composed in-memory without backing YAML files) have no real config file paths to
// offer — this still gives every run a distinct, deterministic output directory.
CompositionFilePaths buildSyntheticFilePaths(const types::SimulationCompositionData& composition) {

    CompositionFilePaths file_paths;
    for (std::size_t sim_index = 0;
         sim_index < composition.simulation_mission_groups.size();
         ++sim_index) {

        const auto& missions = std::get<1>(composition.simulation_mission_groups[sim_index]);

        std::vector<ReferencedConfigFile> mission_refs;
        mission_refs.reserve(missions.size());

        for (std::size_t mission_index = 0;
             mission_index < missions.size();
             ++mission_index) {

            mission_refs.push_back(ReferencedConfigFile{
                "mission_" + std::to_string(sim_index) + "_" +
                    std::to_string(mission_index),
                std::nullopt});
        }

        file_paths.simulation_mission_paths.emplace_back(
            ReferencedConfigFile{
                "sim_" + std::to_string(sim_index),
                std::nullopt},
            std::move(mission_refs));
    }

    for (std::size_t drone_index = 0;
         drone_index < composition.drone_configs.size();
         ++drone_index) {

        file_paths.drone_paths.push_back(
            "drone_" + std::to_string(drone_index));
    }

    for (std::size_t lidar_index = 0;
         lidar_index < composition.lidar_configs.size();
         ++lidar_index) {

        file_paths.lidar_paths.push_back(
            "lidar_" + std::to_string(lidar_index));
    }

    return file_paths;
}

// Builds `output_path/simulations/<sim>/<mission>/<drone>__<lidar>` and disambiguates against
// `used_leaf_dirs` (e.g. the same mission file referenced twice in one composition) by appending
// "__2", "__3", ... to the leaf component, so no run ever silently overwrites another run's
// map_output.npy within the same SimulationManager::run() invocation.
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

std::string contextLabel(const std::string& sim_stem, const std::string& mission_stem, const std::string& drone_stem,
                         const std::string& lidar_stem) {
    return "sim=" + sim_stem + " mission=" + mission_stem + " drone=" + drone_stem + " lidar=" + lidar_stem;
}

// Builds the per-run SimulationResult for a run that never actually ran -- either because
// run_factory_->create()/run->run() threw, or because `simulation`/`mission` was already known
// unusable before any of that was attempted (ConfigLoader's load_error placeholders). Per the
// assignment's Error Handling Policy (Assignment 2: "if an error occurs during the run and the
// simulator can continue to the next scenario, the failed scenario should get the score -1 ...
// the simulation should continue"), this lets the surrounding loop keep going instead of letting
// a failure abort the whole composition.
//
// output_map_config.resolution is computed via outputMapResolution() (the same formula
// SimulationRunFactoryImpl::outputMapConfig() uses: simulation.map_resolution) rather than left
// default-zero, so SimulationOutputWriter's buildMissionNode() -- which reads resolution_cm off
// whichever run it consumes first for a mission -- reports the resolution this run was actually
// configured against, not a meaningless 0, when real simulation data was available. For a
// load_error placeholder simulation (never actually parsed), this naturally evaluates to 0 -- an
// honest reflection that no real hidden-map resolution was ever known, not a fabricated number.
// resolution_request_status is left at its default (Ignored): there is no achieved-vs-requested
// comparison to make when no map was built.
types::SimulationResult buildErrorResult(const types::SimulationConfigData& simulation,
                                        const common_types::MissionConfigData& mission,
                                        common_types::ErrorRef error) {
    types::SimulationResult result;
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.output_map_config.boundaries = mission.mission_bounds;
    result.output_map_config.offset =
        Position3D{mission.mission_bounds.min_x, mission.mission_bounds.min_y, mission.mission_bounds.min_height};
    result.output_map_config.resolution = outputMapResolution(simulation);
    result.mission_score = -1.0;

    common_types::MissionRunResult mission_result;
    mission_result.status = common_types::MissionRunStatus::Error;
    mission_result.steps = 0;
    mission_result.errors.push_back(std::move(error));
    result.mission_results = {mission_result};
    return result;
}

// Overload for the run_factory_->create()/run->run() exception path: derives the ErrorRef from
// the caught exception (the specific code if it's our SimulationException, else a generic
// fallback for any other std::exception) and forwards to the core overload above.
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

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    return runInternal(composition, output_path, buildSyntheticFilePaths(composition));
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path,
                                                      const CompositionFilePaths& file_paths) {
    return runInternal(composition, output_path, file_paths);
}

types::SimulationManagerReport SimulationManager::runInternal(const types::SimulationCompositionData& composition,
                                                              const std::filesystem::path& output_path,
                                                              const CompositionFilePaths& file_paths) {
    std::vector<types::SimulationResult> runs;
    std::set<std::filesystem::path> used_leaf_dirs;

    for (std::size_t sim_index = 0; sim_index < composition.simulation_mission_groups.size(); ++sim_index) {
        const auto& [simulation, missions] = composition.simulation_mission_groups[sim_index];
        const auto& [sim_ref, mission_refs] = file_paths.simulation_mission_paths[sim_index];
        const std::string sim_stem = stem(sim_ref.path);

        for (std::size_t mission_index = 0; mission_index < missions.size(); ++mission_index) {
            const common_types::MissionConfigData& mission = missions[mission_index];
            const ReferencedConfigFile& mission_ref = mission_refs[mission_index];
            const std::string mission_stem = stem(mission_ref.path);

            for (std::size_t drone_index = 0; drone_index < composition.drone_configs.size(); ++drone_index) {
                const common_types::DroneConfigData& drone = composition.drone_configs[drone_index];
                const std::string drone_stem = stem(file_paths.drone_paths[drone_index]);


                for (std::size_t lidar_index = 0; lidar_index < composition.lidar_configs.size(); ++lidar_index) {
                    const common_types::LidarConfigData& lidar = composition.lidar_configs[lidar_index];
                    const std::string lidar_stem = stem(file_paths.lidar_paths[lidar_index]);

                    const std::filesystem::path leaf_dir =
                        uniqueLeafDir(output_path, sim_stem, mission_stem, drone_stem, lidar_stem, used_leaf_dirs);
                    std::filesystem::create_directories(leaf_dir);

                    const CerrContextGuard cerr_guard(contextLabel(sim_stem, mission_stem, drone_stem, lidar_stem));
                    // ConfigLoader already determined this group/mission is unusable (a bad
                    // simulation_config or mission_config file) -- score -1 without ever calling
                    // the factory, checking the group-level marker first since it implies every
                    // mission under it is unusable regardless of that mission's own load_error.
                    if (sim_ref.load_error) {
                        std::cerr << "SimulationManager::run: simulation_config failed to load, scoring -1: "
                            << sim_ref.load_error->message << '\n';

                        runs.push_back(buildErrorResult(simulation, mission, *sim_ref.load_error));

                    } else if (mission_ref.load_error) {
                        std::cerr << "SimulationManager::run: mission_config failed to load, scoring -1: "
                            << mission_ref.load_error->message << '\n';

                        runs.push_back(buildErrorResult(simulation, mission, *mission_ref.load_error));
                    } else {
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
