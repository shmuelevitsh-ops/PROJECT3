// Migrated from Project 2 (FILES PROJECT 2/tests/integration/full_flow_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Scenarios, compositions, max_steps, and assertions are
// unchanged from the Project 2 original (PROJ2_TESTS_PLAN.md §9.3/§9.4,
// PROJ2_TESTS_EXECUTION_PLAN.md Phase 6 task 2) -- these are the tests that may
// run slowly (or hit a safety-ceiling timeout) under LID03/DRO08-class bugs, but
// that risk is covered by the fast, focused component tests added in Phase 1/5
// (mock_lidar_test.cpp's ScanOrientationArgumentDeterminesBeamDirection,
// drone_control_test.cpp's ElevateMovementForwardsNegativeDistanceUnchanged),
// not by narrowing this file's own coverage.
//
// NON-MECHANICAL ADAPTATION for every test that spawns the compiled binary:
// Project 3's Simulator CLI is not Project 2's `drone_mapper_simulation
// <composition> <output_dir>` contract -- it is `-comparative simulation=<file>
// mission_control_folder=<dir> algorithm=<file>`, where mission_control_folder is
// scanned for candidate .so plugins and the results directory
// (`comparative_results_<timestamp>`) is auto-created by the binary itself
// directly under that folder; there is no caller-specified output path at all
// (CliOptions.cpp/SimulatorRunner.cpp, built in an earlier Project 3 stage per
// SIMULATOR_CORE_PLAN.md). This was only discovered while running these tests
// here, since no earlier migration phase ever spawned the actual compiled
// binary. Every subprocess-spawning test below now runs -comparative mode
// against a scratch mission_control_folder containing exactly one copy of the
// real, freshly-built MissionControl_322889890_315113738.so (so each run
// produces exactly one component's worth of output, preserving each test's
// original "one composition run, one result" shape) and locates the created
// comparative_results_* directory to find error.log / per-component YAML / map
// output. SimulationManager/ConfigLoader/SimulationOutputWriter's own behavior --
// the actual thing every scenario below protects -- is completely unchanged;
// writeSimulationOutput()'s YAML schema is documented unchanged from Assignment 2
// (SimulationOutputWriter.h), so every score_report/summary assertion below is
// unchanged from the Project 2 original. No scenario, composition, max_steps, or
// assertion was narrowed, rewritten, or weakened.
//
// Additional Project-3 mechanical adaptations: namespaces/includes across three
// modules (MissionControl, Algorithm, Simulator), and — for
// ScriptedMockAlgorithmRealHappyPathWritesMapAndScoresNonZero only (the one test
// that does not spawn the binary at all) — the same MissionControlImpl DI shape
// change already applied throughout §7.2 (MissionControlImpl now builds its own
// DroneControlImpl internally from common::MissionControlDependencies; Project 2
// built one by hand and passed it in).
//
// NOT migrated: MapsComparisonCliWithNoConfigAssumesSameDefaultGeometryForBothMaps
// (10th test in the Project 2 original). It exercises maps_comparison_main.cpp's
// CLI entry point (drone_mapper::run(argc, argv, out, err)) compiled directly into
// the test binary. Project 3 has no maps_comparison CLI executable / *_main.cpp
// equivalent anywhere in the tree — confirmed during Phase 2 for the dedicated
// maps_comparison_cli_test.cpp suite (12 tests, also not migrated for the same
// reason; see MIGRATION_MANIFEST.md) and reconfirmed here: there is nothing to
// link this test's drone_mapper::run() call against. This is an infrastructure
// gap, not a scenario change -- revisit both if/when such a target is added to
// Project 3.
//
// NOT migrated: BinaryCliInvocationWithOmittedOutputDirDefaultsToCurrentWorkingDirectory
// (also from the Project 2 original). It exercised README.md's old
// `[<simulation.yaml>] [<output_path>]` CLI contract, specifically that an
// omitted output-dir positional argument defaults to the invocation's current
// working directory. Project 3's real CLI has no positional arguments and no
// such default at all -- every argument is a mandatory key=value, and the
// results directory is always auto-computed under mission_control_folder/
// algorithms_folder, never influenced by the caller's CWD. The specific CLI
// feature this test protects no longer exists in Project 3 to protect.

// Integration tests exercise the real benchmark house
// (data_maps/benchmark_map_normalized.npy, or a small real sub-region of it)
// end-to-end through real components.
//
// Most tests spawn the built `simulator_322889890_315113738` binary in
// -comparative mode and inspect the produced comparative_results_* directory's
// error.log, per-component simulation_output_<stem>.yaml, and the documented
// <component>/simulations/<sim>/<mission>/<drone>__<lidar>/ map hierarchy. The
// scripted mock-algorithm test wires components directly so it can inject a
// fixed command sequence.

#include <Algorithm/MappingAlgorithmImpl.h>
#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/MissionControlImpl.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>

#include "mocks/GMockIMappingAlgorithm.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <sys/wait.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifndef TEST_INPUTS_DIR
#define TEST_INPUTS_DIR "."
#endif

#ifndef DATA_MAPS_DIR
#define DATA_MAPS_DIR "."
#endif

#ifndef DRONE_MAPPER_SIMULATION_BINARY
#define DRONE_MAPPER_SIMULATION_BINARY "./simulator_322889890_315113738"
#endif

#ifndef ALGORITHM_PLUGIN_PATH
#define ALGORITHM_PLUGIN_PATH "./Algorithm_322889890_315113738.so"
#endif

#ifndef MISSION_CONTROL_PLUGIN_PATH
#define MISSION_CONTROL_PLUGIN_PATH "./MissionControl_322889890_315113738.so"
#endif

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace MissionControl_322889890_315113738;
using namespace Algorithm_322889890_315113738;

namespace {

std::filesystem::path scratchDir() {
    // Relative to CWD, not an absolute path -- matches the established
    // convention (test binary run from the repo root) already relied on by
    // every map_filename string in this file. Lets test output be inspected
    // directly in the workspace; see .gitignore for why it's never committed.
    const std::filesystem::path dir = std::filesystem::path("tests/integration/test_output");
    std::filesystem::create_directories(dir);
    return dir;
}

std::filesystem::path testInputsDir() {
    return std::filesystem::path(TEST_INPUTS_DIR);
}

// The stem shared by every scenario's single candidate component -- always a copy of the same
// real MissionControl_322889890_315113738.so (see runComparative() below), so every
// per-component YAML this file inspects is always named "simulation_output_<this>.yaml", and
// every map output lives under "<results_dir>/<this>/simulations/...".
std::string missionControlStem() {
    return std::filesystem::path(MISSION_CONTROL_PLUGIN_PATH).stem().string();
}

struct RunOutcome {
    bool exited_normally = false;
    int exit_code = -1;
    std::string stderr_text;
    // The one comparative_results_<timestamp> directory the binary created directly under this
    // run's mission_control_folder, if any -- empty if the run never got that far.
    std::filesystem::path results_dir;
};

// Spawns the built simulator_322889890_315113738 binary in -comparative mode against
// `composition_path`, using a dedicated, freshly-cleared mission_control_folder (under
// scratchDir()/mission_control_libs/<tag>/) containing exactly one candidate .so -- a copy of the
// real, freshly-built MissionControl plugin -- so this run always produces exactly one
// component's worth of output, matching this file's original "one composition run, one result"
// scenarios. See the file banner comment for why this replaced Project 2's simple <composition>
// <output_dir> CLI.
RunOutcome runComparative(const std::string& tag, const std::filesystem::path& composition_path) {
    const std::filesystem::path binary_path = DRONE_MAPPER_SIMULATION_BINARY;
    if (!std::filesystem::exists(binary_path)) {
        ADD_FAILURE() << "build the project before running this test: " << binary_path;
        return {};
    }
    if (!std::filesystem::exists(composition_path)) {
        ADD_FAILURE() << "missing composition fixture: " << composition_path;
        return {};
    }

    const std::filesystem::path scenario_dir = scratchDir() / "mission_control_libs" / tag;
    std::filesystem::remove_all(scenario_dir);
    std::filesystem::create_directories(scenario_dir);
    const std::filesystem::path plugin = MISSION_CONTROL_PLUGIN_PATH;
    std::filesystem::copy_file(plugin, scenario_dir / plugin.filename());

    const std::filesystem::path stderr_path = scratchDir() / (tag + "_stderr.log");
    const std::string command = "\"" + binary_path.string() + "\" -comparative simulation=\"" +
                                 composition_path.string() + "\" mission_control_folder=\"" + scenario_dir.string() +
                                 "\" algorithm=\"" + std::string(ALGORITHM_PLUGIN_PATH) + "\" 2> \"" +
                                 stderr_path.string() + "\"";
    const int raw_status = std::system(command.c_str());

    RunOutcome outcome;
    outcome.exited_normally = WIFEXITED(raw_status);
    if (outcome.exited_normally) {
        outcome.exit_code = WEXITSTATUS(raw_status);
    }
    {
        std::ifstream in(stderr_path);
        std::ostringstream out;
        out << in.rdbuf();
        outcome.stderr_text = out.str();
    }
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(scenario_dir)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("comparative_results_", 0) == 0) {
            outcome.results_dir = entry.path();
            break;
        }
    }
    return outcome;
}

DroneConfigData droneConfigWithRadius(double radius_cm) {
    DroneConfigData config;
    config.radius = radius_cm * isq::length[cm];
    config.max_rotate = 90.0 * horizontal_angle[deg];
    config.max_advance = 10.0 * isq::length[cm];
    config.max_elevate = 10.0 * isq::length[cm];
    return config;
}

// Helpers for a small real sub-region of the benchmark house
// (data_maps/benchmark_map_normalized.npy, shape 29x30x31 @ 10cm/voxel).
// Used by the scripted mock-algorithm test below, which constructs its own
// Map3DImpl/MissionControlImpl chain by hand.
// DATA_MAPS_DIR is an absolute, CMake-provided path (see CMakeLists.txt) so this resolves
// correctly regardless of the test binary's current working directory.
std::filesystem::path benchmarkMapFilename() {
    return std::filesystem::path(DATA_MAPS_DIR) / "benchmark_map_normalized.npy";
}

std::shared_ptr<NpyArray> loadBenchmarkMap() {
    auto array = std::make_shared<NpyArray>();
    const char* error = array->LoadNPY(benchmarkMapFilename().string());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load benchmark map: ") + error);
    }
    return array;
}

// Mirrors SimulationRunFactoryImpl::hiddenMapConfig(): boundaries span the
// loaded array's full physical shape, zero offset, real map resolution.
MapConfig benchmarkHiddenMapConfig(const NpyArray::shape_t& shape) {
    constexpr double kResolutionCm = 10.0;
    return MapConfig{
        MappingBounds{
            0.0 * x_extent[cm], static_cast<double>(shape[0]) * kResolutionCm * x_extent[cm],
            0.0 * y_extent[cm], static_cast<double>(shape[1]) * kResolutionCm * y_extent[cm],
            0.0 * z_extent[cm], static_cast<double>(shape[2]) * kResolutionCm * z_extent[cm]},
        Position3D{},
        kResolutionCm * isq::length[cm]};
}

// Mirrors SimulationRunFactoryImpl::outputMapConfig(): offset = boundaries.min
// on every axis, so the fresh output array never needs a negative index.
MapConfig benchmarkOutputMapConfig(const MissionConfigData& mission) {
    return MapConfig{
        mission.mission_bounds,
        Position3D{mission.mission_bounds.min_x, mission.mission_bounds.min_y, mission.mission_bounds.min_height},
        mission.gps_resolution};
}

LidarConfigData benchmarkLidarConfig(double z_max_cm) {
    LidarConfigData config;
    config.z_min = 5.0 * isq::length[cm];
    config.z_max = z_max_cm * isq::length[cm];
    config.d = 2.5 * isq::length[cm];
    config.fov_circles = 2;
    return config;
}

// ── Region B: a 1-voxel-wide (10cm) closet whose only opening is a real 2x1
// roof gap into open sky. Interior x=7 only, y:[180,200)cm, z:[210,270)cm
// (room) capped by a real roof gap at z:[270,280)cm, opening into clear sky
// at z:[280,310)cm. Confirmed via direct .npy inspection: the nearest
// Occupied voxel center on either side of x=7 (at x=6 and x=8) is exactly
// 10cm away -- the tightest passage anywhere in the house.
MissionConfigData regionBMissionConfig() {
    MissionConfigData config;
    config.max_steps = 50;
    config.gps_resolution = 10.0 * isq::length[cm];
    config.output_mapping_resolution_factor = 1.0;
    config.mission_bounds = MappingBounds{
        70.0 * x_extent[cm], 80.0 * x_extent[cm],
        180.0 * y_extent[cm], 200.0 * y_extent[cm],
        210.0 * z_extent[cm], 310.0 * z_extent[cm]};
    return config;
}

} // namespace

// ── Real benchmark house map, full program flow ──────────────────────────────
//
// Parameters match the proof-of-concept run documented in
// docs/mapping_algorithm_benchmark_fixes.md: start zone derived from the map's
// own geometry (the open 4-voxel-gap yard column), drone radius small enough
// for the 1x1 roof opening. Runtime is ~45s on this machine — well within the
// assignment's 60s budget, but notably slower than every other test here,
// since this is the only one exercising the full-scale real map rather than a
// small real sub-region.
//
// Spawns the real simulator_322889890_315113738 binary against a dedicated
// composition (composition_full_house.yaml -> sim_full_house.yaml +
// mission_full_house.yaml + drone_small.yaml + lidar_full_house.yaml), one
// run, distinct from composition_basic.yaml's small sub-regions and
// composition_multilevel.yaml's Region C -- so this validates the full
// program flow (main() -> CerrRedirectGuard -> SimulationManager ->
// SimulationOutputWriter) against the full-scale real map, not just the
// scoring behavior of a library call in isolation.
TEST(Integration, RealBenchmarkHouseFullProgramFlowAchievesFullScoreWithinTimeBudget) {
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_full_house.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("real_benchmark_house_full_flow", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    EXPECT_TRUE(std::filesystem::exists(error_log))
        << "simulator_322889890_315113738 must always create an error.log in the results directory, even "
           "on a fully successful run; missing file: " << error_log;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml))
        << "simulator_322889890_315113738 did not write this component's simulation_output YAML to: " << output_yaml;

    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report) << "the per-component YAML must contain a top-level score_report node";
    ASSERT_EQ(score_report["summary"]["total_runs"].as<std::size_t>(), 1u);

    const YAML::Node run_node = score_report["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run_node["status"].as<std::string>(), "completed");
    const double score = run_node["score"].as<double>();
    EXPECT_GE(score, 95.0) << "Expected the benchmark house to be (near-)fully mapped; got " << score
                            << ". See docs/mapping_algorithm_benchmark_fixes.md for the expected baseline (100.0).";

    // The documented hierarchy: this run's map must land at exactly this nested path, named from
    // composition_full_house.yaml's real config files -- not a flat or differently-shaped path.
    const std::filesystem::path expected_map = run.results_dir / missionControlStem() / "simulations" / "sim_full_house" /
                                               "mission_full_house" / "drone_small__lidar_full_house" /
                                               "map_output.npy";
    EXPECT_TRUE(std::filesystem::exists(expected_map))
        << "expected the full-house run's map at the documented <component>/simulations/<sim>/<mission>/"
           "<drone>__<lidar>/map_output.npy path, but it is missing: " << expected_map;
}

// Exercises SimulationManager's nested loop over simulation/mission groups,
// drones, and lidars. This test checks the cross-product shape and
// physically-grounded score relationships; a separate test checks the output
// YAML schema for the same fixture.
TEST(Integration, CompositionCrossProductCoversEverySimulationMissionDroneLidarCombination) {
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_basic.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("composition_cross_product", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml));
    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report);

    // composition_basic.yaml: sim_1 x 1 mission (mission_a), sim_2 x 2 missions (mission_b,
    // mission_c) == 3 missions total, x 2 drones x 2 lidars == 12 runs -- proves the
    // nested-tuple cross-product shape is honored, not just a flat list.
    constexpr std::size_t kExpectedRuns = 12;
    EXPECT_EQ(score_report["summary"]["total_runs"].as<std::size_t>(), kExpectedRuns);

    const YAML::Node simulations = score_report["simulations"];
    ASSERT_EQ(simulations.size(), 2u) << "one entry per simulation_config group";

    std::vector<double> scores;
    for (const YAML::Node& sim_node : simulations) {
        for (const YAML::Node& mission_node : sim_node["missions"]) {
            const YAML::Node runs = mission_node["runs"];
            // Every (simulation, mission) pair must appear exactly drones*lidars times.
            ASSERT_EQ(runs.size(), 4u) << "(simulation, mission) pair should appear once per drone*lidar combination";

            for (const YAML::Node& run_node : runs) {
                // Happy flow only: every run must complete without an explicit error status, and
                // every score must land in the assignment's valid [0,100] range (the -1 error
                // sentinel should never appear here).
                EXPECT_NE(run_node["status"].as<std::string>(), "error")
                    << "happy-flow-only: no run in this composition should hit an explicit error status";
                const double score = run_node["score"].as<double>();
                EXPECT_GE(score, 0.0);
                EXPECT_LE(score, 100.0);
                scores.push_back(score);
            }

            // The physically-grounded diversity story: for Region D (sim_2's missions, mission_b
            // and mission_c), drone_small (fits the corridor) should score higher than
            // drone_large (blocked at the corridor) under the same mission/lidar -- a relative
            // comparison, not a calibrated absolute number. runs[] is ordered drone_small x
            // lidar_a, drone_small x lidar_b, drone_large x lidar_a, drone_large x lidar_b
            // (buildMissionNode's nested drone-then-lidar loop, matching composition_basic.yaml's
            // drone_configs/lidar_configs list order).
            const std::string mission_config = mission_node["mission_config"].as<std::string>();
            if (mission_config.find("mission_b") != std::string::npos ||
                mission_config.find("mission_c") != std::string::npos) {
                for (std::size_t lidar_offset = 0; lidar_offset < 2; ++lidar_offset) {
                    const double small_score = runs[lidar_offset]["score"].as<double>();
                    const double large_score = runs[2 + lidar_offset]["score"].as<double>();
                    EXPECT_GT(small_score, large_score)
                        << mission_config << ", lidar offset " << lidar_offset
                        << ": drone_small (fits the corridor) should outscore drone_large (blocked at it)";
                }
            }
        }
    }

    // Natural diversity check (no hardcoded values): the 12 runs span two different real regions,
    // two drone sizes, and two lidar ranges, so they must not all land on the exact same score --
    // that would mean the configuration knobs aren't actually doing anything.
    ASSERT_EQ(scores.size(), kExpectedRuns);
    const auto [min_it, max_it] = std::minmax_element(scores.begin(), scores.end());
    EXPECT_LT(*min_it, *max_it) << "expected genuine score variance across drone/lidar/region combinations, not a"
                                    " single repeated value";
}

TEST(Integration, YamlDrivenCompositionProducesExpectedSimulationOutput) {
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_basic.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    // composition_basic.yaml: sim_1 x 1 mission, sim_2 x 2 missions == 3
    // missions total, x 2 drones x 2 lidars == 12 runs.
    constexpr std::size_t kExpectedRuns = 12;

    const RunOutcome run = runComparative(composition_path.stem().string(), composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    EXPECT_TRUE(std::filesystem::exists(error_log))
        << "simulator_322889890_315113738 must always create an error.log in the results directory, even "
           "on a fully successful run; missing file: " << error_log;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml))
        << "simulator_322889890_315113738 did not write this component's simulation_output YAML to: " << output_yaml;

    // The actual point of this output-layout change: every run's map_output.npy must land at the
    // documented <component>/simulations/<sim>/<mission>/<drone>__<lidar>/ path, named after the
    // real config files composition_basic.yaml references -- not a flat, numerically-indexed file.
    const std::filesystem::path component_results = run.results_dir / missionControlStem();
    for (const std::filesystem::path& relative_map :
         {std::filesystem::path("simulations/sim_1/mission_a/drone_small__lidar_a/map_output.npy"),
          std::filesystem::path("simulations/sim_1/mission_a/drone_large__lidar_b/map_output.npy"),
          std::filesystem::path("simulations/sim_2/mission_b/drone_small__lidar_a/map_output.npy"),
          std::filesystem::path("simulations/sim_2/mission_c/drone_large__lidar_b/map_output.npy")}) {
        const std::filesystem::path expected_map = component_results / relative_map;
        EXPECT_TRUE(std::filesystem::exists(expected_map))
            << "expected a map at the documented <component>/simulations/<sim>/<mission>/<drone>__<lidar>/ "
               "path, but it is missing: " << expected_map;
    }

    // ── Structural schema validation against the assignment's documented
    // simulation_output.yaml format -- field presence, types, and fixed
    // contract constants (score_range/error_score come from production
    // constants in SimulationManager.cpp, not measured outcomes, so they are
    // legitimately asserted exactly). No measured score/step value is
    // asserted exactly anywhere below.
    const YAML::Node root = YAML::LoadFile(output_yaml.string());
    const YAML::Node score_report = root["score_report"];
    ASSERT_TRUE(score_report) << "the per-component YAML must contain a top-level score_report node";

    EXPECT_EQ(score_report["composition_file"].as<std::string>(), composition_path.string());
    EXPECT_FALSE(score_report["generated_at_utc"].as<std::string>().empty());
    EXPECT_EQ(score_report["metric"].as<std::string>(), "output_map_accuracy");
    EXPECT_EQ(score_report["score_range"]["min"].as<double>(), 0.0);
    EXPECT_EQ(score_report["score_range"]["max"].as<double>(), 100.0);
    EXPECT_EQ(score_report["error_score"].as<int>(), -1);

    const YAML::Node summary = score_report["summary"];
    ASSERT_TRUE(summary);
    EXPECT_EQ(summary["total_runs"].as<std::size_t>(), kExpectedRuns);
    const std::size_t scored_runs = summary["scored_runs"].as<std::size_t>();
    const std::size_t error_runs = summary["error_runs"].as<std::size_t>();
    EXPECT_EQ(scored_runs + error_runs, kExpectedRuns);
    EXPECT_EQ(error_runs, 0u) << "happy-flow-only: no run in this composition should be an unrecoverable error";

    const double average_score = summary["average_score"].as<double>();
    const double min_score = summary["min_score"].as<double>();
    const double max_score = summary["max_score"].as<double>();
    EXPECT_GE(min_score, 0.0);
    EXPECT_LE(max_score, 100.0);
    EXPECT_LE(min_score, average_score);
    EXPECT_LE(average_score, max_score);
    // Natural diversity (no hardcoded values): 12 runs across 2 real regions,
    // 2 drones, and 2 lidars must not all land on the same score.
    EXPECT_LT(min_score, max_score) << "expected genuine score variance across the composition's drone/lidar/region"
                                        " combinations, not a single repeated value";

    const YAML::Node simulations = score_report["simulations"];
    ASSERT_EQ(simulations.size(), 2u) << "one entry per simulation_config group";
    for (std::size_t sim_index = 0; sim_index < simulations.size(); ++sim_index) {
        const YAML::Node sim_node = simulations[sim_index];
        EXPECT_TRUE(sim_node["simulation_config"]);
        for (const YAML::Node& mission_node : sim_node["missions"]) {
            EXPECT_TRUE(mission_node["mission_config"]);
            EXPECT_EQ(mission_node["resolution_cm"].as<double>(), 10.0);
            const std::string resolution_status = mission_node["resolution_request_status"].as<std::string>();
            EXPECT_TRUE(resolution_status == "ACCEPTED" || resolution_status == "IGNORED" ||
                        resolution_status == "IGNORED_TOO_SMALL");

            const YAML::Node runs = mission_node["runs"];
            ASSERT_EQ(runs.size(), 4u) << "2 drones x 2 lidars per mission";
            for (std::size_t run_offset = 0; run_offset < runs.size(); ++run_offset) {
                const YAML::Node run_node = runs[run_offset];
                ASSERT_TRUE(run_node["drone_config"]);
                ASSERT_TRUE(run_node["lidar_config"]);
                // composition_basic.yaml's drone_configs/lidar_configs lists are
                // [drone_small, drone_large] x [lidar_a, lidar_b]; buildMissionNode's
                // documented "drone outer, lidar inner" emission order means run_offset
                // (0..3) must decompose as drone_index = run_offset / 2, lidar_index =
                // run_offset % 2. This pins each emitted run's drone_config/lidar_config
                // labels to the actual configuration pairing that produced its score --
                // catching a consumption-order mismatch (e.g. buildMissionNode iterating
                // lidar-outer/drone-inner while the underlying runs vector was produced
                // drone-outer/lidar-inner) that a magnitude-only comparison would miss
                // whenever it happens to preserve relative score ordering.
                const std::string expected_drone = (run_offset / 2 == 0) ? "drone_small" : "drone_large";
                const std::string expected_lidar = (run_offset % 2 == 0) ? "lidar_a" : "lidar_b";
                const std::string drone_config = run_node["drone_config"].as<std::string>();
                const std::string lidar_config = run_node["lidar_config"].as<std::string>();
                EXPECT_NE(drone_config.find(expected_drone), std::string::npos)
                    << "run[" << run_offset << "].drone_config ('" << drone_config << "') does not match the "
                       "drone expected at this position under drone-outer/lidar-inner emission order ('"
                    << expected_drone << "') -- the emitted drone_config/lidar_config pairing must correspond "
                       "to the configuration that actually produced this run's score, not a transposed pairing";
                EXPECT_NE(lidar_config.find(expected_lidar), std::string::npos)
                    << "run[" << run_offset << "].lidar_config ('" << lidar_config << "') does not match the "
                       "lidar expected at this position under drone-outer/lidar-inner emission order ('"
                    << expected_lidar << "') -- the emitted drone_config/lidar_config pairing must correspond "
                       "to the configuration that actually produced this run's score, not a transposed pairing";
                // Happy-flow-only: status must never be the explicit "error"
                // outcome. `error_ref` may still legitimately appear on a
                // "completed" run -- e.g. drone_large reaching Region D's
                // hallway but never the room behind its corridor leaves
                // voxels unmappable, which DroneControlImpl reports as a
                // diagnostic error_ref without changing the run's status: a
                // graceful, non-error outcome, not a crash.
                const std::string status = run_node["status"].as<std::string>();
                EXPECT_TRUE(status == "completed" || status == "max_steps")
                    << "happy-flow-only: status was '" << status << "'";
                EXPECT_GE(run_node["score"].as<double>(), 0.0);
                EXPECT_LE(run_node["score"].as<double>(), 100.0);
            }

            // sim_2 (index 1, Region D): runs[] is ordered drone_small x
            // lidar_a, drone_small x lidar_b, drone_large x lidar_a,
            // drone_large x lidar_b (buildMissionNode's nested drone-then-
            // lidar loop, same order as composition_basic.yaml's
            // drone_configs/lidar_configs lists). drone_small should outscore
            // drone_large at the same lidar -- a relative comparison, not a
            // calibrated absolute number.
            if (sim_index == 1) {
                for (std::size_t lidar_offset = 0; lidar_offset < 2; ++lidar_offset) {
                    const double small_score = runs[lidar_offset]["score"].as<double>();
                    const double large_score = runs[2 + lidar_offset]["score"].as<double>();
                    EXPECT_GT(small_score, large_score)
                        << "Region D, lidar offset " << lidar_offset
                        << ": drone_small (fits the corridor) should outscore drone_large (blocked at it)";
                }
            }
        }
    }
}

// Internal.MockAlgorithmAlwaysHoveringHitsMaxSteps (tests/integration/internal_flow_test.cpp)
// only proves MissionControlImpl's MaxSteps termination plumbing -- a drone
// that never moves never scans or writes to output_map. This test is the
// canonical "mock algorithm, everything else real" happy path, exercised
// against Region B -- the tightest real passage in the house (10cm clear
// width) capped by a real roof opening: a short, scripted
// MappingStepCommand sequence (elevate within the closet, scan upward
// toward the roof gap, finish) driven through real
// MissionControlImpl/Map3DImpl (which builds its own real DroneControlImpl
// internally, §7.2), with MappingAlgorithmImpl substituted by a GMock
// returning the fixed sequence.
//
// SimulationRunFactoryImpl::create() always constructs MappingAlgorithmImpl,
// so this test wires the production components directly to inject the mock
// command sequence while still exercising real movement, lidar, map, and
// mission-control behavior.
TEST(Integration, ScriptedMockAlgorithmRealHappyPathWritesMapAndScoresNonZero) {
    const std::shared_ptr<NpyArray> benchmark_array = loadBenchmarkMap();
    auto hidden_map = std::make_unique<Map3DImpl>(benchmark_array, benchmarkHiddenMapConfig(benchmark_array->Shape()));

    const MissionConfigData mission = regionBMissionConfig(); // x:[70,80) y:[180,200) z:[210,310) cm
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), benchmarkOutputMapConfig(mission));

    const DroneConfigData drone = droneConfigWithRadius(4.0);
    const LidarConfigData lidar = benchmarkLidarConfig(20.0);

    // Start at (75,185,215)cm -- inside the closet's lower interior (x=7,
    // y=18 voxel), well clear of the real walls at x=6/x=8 and y=17/y=20
    // (each exactly 10cm from this center). Facing east; altitude irrelevant
    // for movement.
    MockGPS gps(Position3D{75.0 * x_extent[cm], 185.0 * y_extent[cm], 215.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, mission.gps_resolution);
    MockMovement movement(gps);
    MockLidar lidar_sensor(lidar, *hidden_map, gps);

    ::testing::NiceMock<test::GMockIMappingAlgorithm> algorithm(
        MappingAlgorithmDependencies{mission, lidar, drone, *output_map});
    {
        ::testing::InSequence seq;
        // Step 1: elevate 10cm (== max_elevate) up the shaft, still well
        // inside the closet (215cm -> 225cm, interior spans 210-270cm).
        EXPECT_CALL(algorithm, nextStep(::testing::_, ::testing::IsNull()))
            .WillOnce(::testing::Return(MappingStepCommand{
                MovementCommand{MovementCommandType::Elevate, RotationDirection::Left,
                                HorizontalAngle{}, 10.0 * isq::length[cm]},
                std::nullopt, AlgorithmStatus::Working}));
        // Step 2: scan straight up (altitude=90deg) toward the roof opening
        // -- in a 10cm-wide shaft, most non-center beams hit the adjacent
        // walls almost immediately, which is exactly the point: the scan
        // result should reflect just how tight this passage really is.
        EXPECT_CALL(algorithm, nextStep(::testing::_, ::testing::IsNull()))
            .WillOnce(::testing::Return(MappingStepCommand{
                std::nullopt, Orientation{0.0 * horizontal_angle[deg], 90.0 * altitude_angle[deg]},
                AlgorithmStatus::Working}));
        EXPECT_CALL(algorithm, nextStep(::testing::_, ::testing::NotNull()))
            .WillOnce(::testing::Return(
                MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Finished}));
    }

    // Dedicated subfolder for this direct component-level integration path.
    const std::filesystem::path output_dir = scratchDir() / "scripted_mock_algorithm_happy_path";
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path output_file = output_dir / "map_output.npy";
    MissionControlImpl mission_control(MissionControlDependencies{
        mission, drone, lidar_sensor, gps, movement, *output_map, algorithm, output_file});

    const MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed)
        << "expected Completed (not MaxSteps), since the scripted algorithm signals Finished on step 3";
    EXPECT_EQ(result.steps, 3u);

    bool any_voxel_left_unmapped_state = false;
    for (long ix = 0; ix < 1 && !any_voxel_left_unmapped_state; ++ix) {
        for (long iy = 0; iy < 2 && !any_voxel_left_unmapped_state; ++iy) {
            for (long iz = 0; iz < 10 && !any_voxel_left_unmapped_state; ++iz) {
                const Position3D voxel_center{
                    (70.0 + (static_cast<double>(ix) + 0.5) * 10.0) * x_extent[cm],
                    (180.0 + (static_cast<double>(iy) + 0.5) * 10.0) * y_extent[cm],
                    (210.0 + (static_cast<double>(iz) + 0.5) * 10.0) * z_extent[cm]};
                if (output_map->atVoxel(voxel_center) != VoxelOccupancy::Unmapped) {
                    any_voxel_left_unmapped_state = true;
                }
            }
        }
    }
    EXPECT_TRUE(any_voxel_left_unmapped_state)
        << "the scripted scan should have written at least one voxel into output_map";

    // Round-trips the just-saved map_output.npy back through a fresh Map3DImpl (the same way any
    // downstream consumer/grader would open it) and checks every probed voxel still reports the
    // same occupancy as the in-memory output_map immediately after the mission ran. Region B's
    // bounds give an output map shaped (1, 2, 10) in (x, y, z) -- deliberately asymmetric in x vs
    // y -- so a serialization bug that writes the array with transposed (y, x, z) axis order is
    // directly observable here: reloading with the original (x, y, z)-shaped MapConfig either
    // throws on the now-mismatched (2, 1, 10) array or silently reads back the wrong voxels.
    {
        auto reloaded_array = std::make_shared<NpyArray>();
        const char* load_error = reloaded_array->LoadNPY(output_file.string());
        ASSERT_EQ(load_error, nullptr) << "failed to reload saved map_output.npy: "
                                        << (load_error != nullptr ? load_error : "");
        const Map3DImpl reloaded_map(reloaded_array, benchmarkOutputMapConfig(mission));

        for (long ix = 0; ix < 1; ++ix) {
            for (long iy = 0; iy < 2; ++iy) {
                for (long iz = 0; iz < 10; ++iz) {
                    const Position3D voxel_center{
                        (70.0 + (static_cast<double>(ix) + 0.5) * 10.0) * x_extent[cm],
                        (180.0 + (static_cast<double>(iy) + 0.5) * 10.0) * y_extent[cm],
                        (210.0 + (static_cast<double>(iz) + 0.5) * 10.0) * z_extent[cm]};
                    EXPECT_EQ(reloaded_map.atVoxel(voxel_center), output_map->atVoxel(voxel_center))
                        << "voxel (x=" << ix << ", y=" << iy << ", z=" << iz
                        << ") read back from the saved map_output.npy does not match the in-memory "
                           "occupancy observed right after the mission ran -- the saved .npy's axis "
                           "ordering must match the in-memory (x, y, z) layout";
                }
            }
        }
    }

    const std::vector<double> scores = MapsComparison::compare(*hidden_map, {output_map.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GT(scores[0], 0.0) << "a real scan against real wall/floor data should score above zero, even"
                                  " though only a small fraction of the closet was explored";
}

// Spawns the real simulator_322889890_315113738 binary against a real composition
// over Region C, a multi-level, floor-crossing real sub-region. This verifies
// the binary contract and output layout.
TEST(Integration, BinaryCliInvocationOnRealCompositionProducesDocumentedOutputLayout) {
    // Region C is covered by a dedicated minimal composition because this
    // test validates the CLI contract, not scoring diversity.
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_multilevel.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("binary_cli_invocation", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    EXPECT_TRUE(std::filesystem::exists(error_log))
        << "simulator_322889890_315113738 must always create an error.log in the results directory, even "
           "on a fully successful run; missing file: " << error_log;
    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    EXPECT_TRUE(std::filesystem::exists(output_yaml))
        << "simulator_322889890_315113738 did not write this component's simulation_output YAML to: " << output_yaml;

    // composition_multilevel.yaml: sim_3 x mission_d x drone_small x lidar_a = 1 run, so exactly
    // this one path should exist -- the documented <component>/simulations/<sim>/<mission>/
    // <drone>__<lidar>/map_output.npy layout, named after the real config files, not a flat
    // numerically-indexed file.
    const std::filesystem::path expected_map = run.results_dir / missionControlStem() / "simulations" / "sim_3" /
                                               "mission_d" / "drone_small__lidar_a" / "map_output.npy";
    EXPECT_TRUE(std::filesystem::exists(expected_map))
        << "expected the Region C run's map at the documented <component>/simulations/<sim>/<mission>/"
           "<drone>__<lidar>/ path, but it is missing: " << expected_map;
}

// Reads a whole text file into a string, for substring-checking error.log contents below.
std::string readFileContents(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

// error.log must capture every std::cerr message logged anywhere across the simulation
// lifecycle -- not just from the SimulationManager::run() phase -- including parse-time errors
// logged by ConfigLoader::parseCompositionData() before run() is ever invoked. Exercises this with
// a composition whose lone simulation_config group references a non-existent file: ConfigLoader
// logs a "failed to load referenced file" diagnostic via std::cerr while parsing, strictly before
// SimulationManager::run() starts. A regression that narrows the CerrRedirectGuard's scope to only
// wrap the run() call (e.g. during a refactor that splits "run" and "write output" into separate
// top-level steps) would let this parse-time diagnostic leak to the real stderr instead of
// landing in the results directory's error.log.
TEST(Integration, ErrorLogCapturesParseTimeDiagnosticsLoggedBeforeSimulationManagerRun) {
    const std::filesystem::path composition_path =
        testInputsDir() / "compositions" / "composition_malformed_simulation_config.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("malformed_simulation_config", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    ASSERT_TRUE(std::filesystem::exists(error_log)) << "missing error.log: " << error_log;

    // Searches specifically for ConfigLoader's own "parseCompositionData: failed to load
    // referenced file" diagnostic -- not just any mention of the broken filename -- since
    // SimulationManager::run() independently logs its own "simulation_config failed to load"
    // message (which does remain inside the guard's scope even under the bug this test targets,
    // and would otherwise make this assertion pass for the wrong reason).
    const std::string error_log_contents = readFileContents(error_log);
    EXPECT_NE(error_log_contents.find("parseCompositionData: failed to load referenced file"), std::string::npos)
        << "error.log must contain the parse-time diagnostic logged by ConfigLoader::parseCompositionData() "
           "before SimulationManager::run() is ever called -- it must not be confined to errors logged "
           "during the run phase. Actual error.log contents:\n"
        << error_log_contents;

    // The affected group's run must still be reported as an explicit error (score -1), not
    // silently dropped, regardless of where its diagnostic message was logged.
    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    const YAML::Node root = YAML::LoadFile(output_yaml.string());
    const YAML::Node summary = root["score_report"]["summary"];
    EXPECT_EQ(summary["error_runs"].as<std::size_t>(), 1u);

    // Pin down *why* it's an error, not just *that* it is: SimulationManager must recognize the
    // group-level load failure up front (via the simulation placeholder's load_error marker) and
    // skip the run factory entirely for every run under that group, reporting the explicit
    // "SIMULATION_CONFIG_LOAD_FAILED" code -- never invoking the real factory/mission pipeline on
    // a default-constructed, never-actually-parsed simulation/mission pair. A regression that
    // drops the load_error marker (while still default-constructing the placeholders) lets this
    // default-constructed pair reach the real factory instead of being skipped; in this fixture
    // that incidentally still ends up scored -1, but via a wrong, misleading error code
    // ("INVALID_RESOLUTION" from the factory's own validation), not the documented group-failure
    // code -- a real downstream consumer relying on the error code to distinguish "config didn't
    // exist" from "mission validation failed" would be misled.
    const YAML::Node run_node = root["score_report"]["simulations"][0]["missions"][0]["runs"][0];
    ASSERT_TRUE(run_node["error_ref"]) << "expected an error_ref on the load-failed group's run";
    EXPECT_EQ(run_node["error_ref"]["code"].as<std::string>(), "SIMULATION_CONFIG_LOAD_FAILED")
        << "a simulation_config group that fails to load must be recognized up front and skip the "
           "run factory entirely -- not fall through to the factory on a default-constructed, "
           "trivially-configured placeholder";
}

// summary.average_score/min_score/max_score must be computed only over successfully-scored runs
// (score >= 0), never dragged toward the -1 error sentinel by runs that failed outright. Exercises
// this with a composition mixing one real, successfully-scored run (sim_1/mission_a, a real region
// of the benchmark house) against one run from a group whose simulation_config doesn't exist (an
// explicit, unrecoverable load failure scored -1) -- so a regression that lets error-scored runs
// leak into the average/min/max aggregate is directly observable: with exactly one real score (100,
// a perfect score for this small, fully-coverable region) and one error (-1), a buggy aggregate
// that includes the error would report average_score 49.5 and min_score -1, instead of the correct
// average_score 100 and min_score 100.
TEST(Integration, SummaryAverageMinMaxExcludeErrorScoredRunsWhenMixedWithSuccessfulRuns) {
    const std::filesystem::path composition_path =
        testInputsDir() / "compositions" / "composition_mixed_success_and_error.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("mixed_success_and_error", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    const YAML::Node root = YAML::LoadFile(output_yaml.string());
    const YAML::Node summary = root["score_report"]["summary"];
    ASSERT_TRUE(summary);

    ASSERT_EQ(summary["total_runs"].as<std::size_t>(), 2u);
    ASSERT_EQ(summary["scored_runs"].as<std::size_t>(), 1u)
        << "expected exactly one successfully-scored run (sim_1/mission_a) and one error run "
           "(the missing simulation_config group)";
    ASSERT_EQ(summary["error_runs"].as<std::size_t>(), 1u);

    // sim_1/mission_a covers a small, fully-explorable real region, so its real run scores a
    // perfect 100 -- the only successfully-scored run in this composition. With that single
    // scored run, average/min/max must all equal it exactly, regardless of the error run's -1.
    EXPECT_DOUBLE_EQ(summary["average_score"].as<double>(), 100.0)
        << "summary.average_score must be computed only over successfully-scored runs (score >= "
           "0) and must not be dragged toward the -1 error sentinel by the error run mixed into "
           "this composition";
    EXPECT_DOUBLE_EQ(summary["min_score"].as<double>(), 100.0)
        << "summary.min_score must be computed only over successfully-scored runs and must not "
           "report the error run's -1 sentinel";
    EXPECT_DOUBLE_EQ(summary["max_score"].as<double>(), 100.0)
        << "summary.max_score must be computed only over successfully-scored runs";
}

// A mission config that omits output_mapping_resolution_factor entirely must default to a factor
// of 1 (output resolution == gps_resolution_cm, request ACCEPTED) end-to-end through the written
// YAML -- not silently default to a factor so small it gets treated as IGNORED TOO SMALL.
// mission_a_no_resolution_factor.yaml is identical to mission_a.yaml (a real, fully-explorable
// sub-region of the benchmark house) except the field is left out entirely, isolating exactly the
// "field absent" parsing path -- as opposed to "field present but < 1", which is already covered
// at the unit level by SimulationRun.ResolutionRequestStatusIgnoredTooSmallWhenFactorBelowOne but
// can't observe ConfigLoader's own default-value parsing since that test constructs
// MissionConfigData directly, never going through ConfigLoader at all.
TEST(Integration, MissionConfigWithOmittedResolutionFactorDefaultsToAcceptedNotIgnoredTooSmall) {
    const std::filesystem::path composition_path =
        testInputsDir() / "compositions" / "composition_missing_resolution_factor.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("missing_resolution_factor", composition_path);
    ASSERT_TRUE(run.exited_normally);
    ASSERT_EQ(run.exit_code, 0) << "stderr: " << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr: " << run.stderr_text;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    const YAML::Node root = YAML::LoadFile(output_yaml.string());
    const YAML::Node mission_node = root["score_report"]["simulations"][0]["missions"][0];
    ASSERT_TRUE(mission_node);

    EXPECT_EQ(mission_node["resolution_request_status"].as<std::string>(), "ACCEPTED")
        << "omitting output_mapping_resolution_factor must default it to 1 (ACCEPTED), not a "
           "value small enough to be treated as IGNORED TOO SMALL";
    EXPECT_DOUBLE_EQ(mission_node["resolution_cm"].as<double>(), 10.0)
        << "with the factor defaulted to 1, the output resolution must equal gps_resolution_cm "
           "(10cm in mission_a_no_resolution_factor.yaml)";
}
