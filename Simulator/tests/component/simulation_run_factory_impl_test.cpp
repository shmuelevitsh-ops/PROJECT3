// Migrated from Project 2 (FILES PROJECT 2/tests/components/simulation_run_factory_impl_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespaces/includes, plus one shape change forced by the new architecture --
// SimulationRunFactoryImpl is no longer default-constructible: Project 2 statically
// linked its mapping algorithm/mission control, but Project 3 always builds them via
// injected factories (Algorithm/MissionControl are dlopen'd plugins, §3). Both tests below
// supply minimal factories local to this file instead of the real
// MappingAlgorithmImpl/MissionControlImpl, keeping this Simulator-owned test decoupled
// from the Algorithm/MissionControl modules -- with one behavior preserved deliberately:
// these tests assert that map_output.npy exists on disk after run(), which in the real
// pipeline comes from MissionControlImpl::runMission() unconditionally calling
// output_map_.save(output_map_file_) before returning (MissionControlImpl.cpp), not from
// anything in SimulationRunFactoryImpl/SimulationRunImpl itself. A pure no-op
// IMissionControl (tried first) made both tests fail -- not because of a Project 3
// production bug, but because it silently dropped that save-on-completion contract the
// tests actually depend on. SavingMissionControl below reproduces just that one documented
// behavior directly against the injected output_map, without needing the full
// DroneControlImpl/algorithm loop max_steps=0 was chosen to avoid. No assertions changed
// from the Project 2 original.

#include <Simulator/SimulationException.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;

#ifndef DATA_MAPS_DIR
#define DATA_MAPS_DIR "."
#endif

namespace {

// A real, loadable hidden map (5x5x5 at 10cm resolution) -- used so create()/run() exercise
// the normal, successful load path rather than the load-failure path covered separately below.
const std::filesystem::path kValidNpy = std::filesystem::path(DATA_MAPS_DIR) / "single_voxel_x4_y4_z4.npy";

// A trivial no-op IMappingAlgorithm, used only so SimulationRunFactoryImpl::create() has
// something valid to construct via its injected factory -- it is never actually invoked
// with max_steps=0 (see comment above; SavingMissionControl below never drives a real
// DroneControlImpl/algorithm loop).
class NoOpMappingAlgorithm final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    MappingStepCommand nextStep(const DroneState&, const LidarScanResult*) override {
        return MappingStepCommand{};
    }
};

// Reproduces MissionControlImpl's one documented behavior these tests actually depend on
// (unconditionally saving the output map before returning -- see the file banner comment
// above) without running a real mission loop.
class SavingMissionControl final : public IMissionControl {
public:
    SavingMissionControl(IMutableMap3D& output_map, std::filesystem::path output_map_file)
        : output_map_(output_map), output_map_file_(std::move(output_map_file)) {}

    MissionRunResult runMission() override {
        output_map_.save(output_map_file_);
        return MissionRunResult{};
    }

private:
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
};

MappingAlgorithmFactory noOpMappingAlgorithmFactory() {
    return [](MappingAlgorithmDependencies dependencies) -> std::unique_ptr<IMappingAlgorithm> {
        return std::make_unique<NoOpMappingAlgorithm>(std::move(dependencies));
    };
}

MissionControlFactory savingMissionControlFactory() {
    return [](MissionControlDependencies dependencies) -> std::unique_ptr<IMissionControl> {
        return std::make_unique<SavingMissionControl>(dependencies.output_map, dependencies.output_map_file);
    };
}

// Minimal-but-valid configs: map_filename must point at a real, loadable hidden map --
// SimulationRunFactoryImpl::create() now throws SimulationException (MAP_LOAD_FAILED) rather than
// falling back to an empty map when the load fails (see SimulationRunFactoryImpl.cpp's
// loadHiddenMap()); that failure path is exercised separately below. max_steps = 0 means
// MissionControlImpl::runMission() never calls into the real DroneControlImpl/MappingAlgorithmImpl
// at all, so this test exercises exactly what it needs to (create()'s map construction + output
// path/filename wiring + the unconditional output_map_.save() call) without depending on the real
// mapping algorithm's behavior on a degenerate single-voxel map.
//
// mission_bounds (absent from the Project 2 original's minimal fixture) is required here: unlike
// Project 2, Project 3's create() builds output_map directly from mission.mission_bounds via a
// real Map3DImpl (outputMapConfig() in SimulationRunFactoryImpl.cpp), which throws on the
// default-zero (degenerate, min==max) bounds a "minimal" MissionConfigData would otherwise carry.
// A real mission always specifies a bounded volume to map, so this is a required-field gap in the
// fixture forced by the new architecture, not a behavioral change to what the test exercises.
SimulationConfigData minimalSimulationConfig() {
    SimulationConfigData config;
    config.map_filename = kValidNpy;
    config.map_resolution = 10.0 * isq::length[cm];
    config.initial_drone_position = Position3D{5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]};
    config.initial_angle = 0.0 * horizontal_angle[deg];
    return config;
}

MissionConfigData minimalMissionConfig() {
    MissionConfigData config;
    config.max_steps = 0;
    config.gps_resolution = 10.0 * isq::length[cm];
    config.output_mapping_resolution_factor = 1.0;
    config.mission_bounds = MappingBounds{
        0.0 * x_extent[cm], 10.0 * x_extent[cm],
        0.0 * y_extent[cm], 10.0 * y_extent[cm],
        0.0 * z_extent[cm], 10.0 * z_extent[cm]};
    return config;
}

DroneConfigData minimalDroneConfig() {
    DroneConfigData config;
    config.radius = 4.0 * isq::length[cm];
    config.max_rotate = 90.0 * horizontal_angle[deg];
    config.max_advance = 10.0 * isq::length[cm];
    config.max_elevate = 10.0 * isq::length[cm];
    return config;
}

LidarConfigData minimalLidarConfig() {
    LidarConfigData config;
    config.z_min = 1.0 * isq::length[cm];
    config.z_max = 20.0 * isq::length[cm];
    config.d = 2.5 * isq::length[cm];
    config.fov_circles = 1;
    return config;
}

} // namespace

TEST(SimulationRunFactoryImpl, CreateAndRunWritesMapOutputUnderTheGivenNestedOutputPath) {
    const std::filesystem::path output_path =
        "out/simulation_run_factory_impl_test/simulations/sim_x/mission_y/drone_a__lidar_b";
    std::filesystem::remove_all(output_path);
    ASSERT_FALSE(std::filesystem::exists(output_path)) << "precondition: directory must not pre-exist";

    SimulationRunFactoryImpl factory(noOpMappingAlgorithmFactory(), savingMissionControlFactory(), /*verbose=*/false);
    const std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulationConfig(), minimalMissionConfig(), minimalDroneConfig(), minimalLidarConfig(), output_path);
    const SimulationResult result = run->run();

    const std::filesystem::path expected_file = output_path / "map_output.npy";
    EXPECT_TRUE(std::filesystem::exists(expected_file))
        << "create()'s previously-nonexistent leaf directory must be created, and the run's map "
           "saved at a fixed filename inside it";
    EXPECT_EQ(result.output_map_file, expected_file);
}

TEST(SimulationRunFactoryImpl, CreateDoesNotDisturbAnExistingSiblingRunsOutputInADifferentDirectory) {
    const std::filesystem::path root = "out/simulation_run_factory_impl_test/sibling_runs";
    std::filesystem::remove_all(root);

    SimulationRunFactoryImpl factory(noOpMappingAlgorithmFactory(), savingMissionControlFactory(), /*verbose=*/false);
    const std::filesystem::path first_dir = root / "drone_a__lidar_a";
    const std::filesystem::path second_dir = root / "drone_b__lidar_a";

    const std::unique_ptr<ISimulationRun> first_run = factory.create(
        minimalSimulationConfig(), minimalMissionConfig(), minimalDroneConfig(), minimalLidarConfig(), first_dir);
    first_run->run();
    const std::unique_ptr<ISimulationRun> second_run = factory.create(
        minimalSimulationConfig(), minimalMissionConfig(), minimalDroneConfig(), minimalLidarConfig(), second_dir);
    second_run->run();

    EXPECT_TRUE(std::filesystem::exists(first_dir / "map_output.npy"))
        << "the first run's output must still exist after a second run targets a sibling directory";
    EXPECT_TRUE(std::filesystem::exists(second_dir / "map_output.npy"));
}

// ── hidden map load failure: create() must fail, not silently substitute an empty map ──────

TEST(SimulationRunFactoryImpl, CreateThrowsMapLoadFailedWhenHiddenMapFileDoesNotExist) {
    SimulationConfigData simulation = minimalSimulationConfig();
    simulation.map_filename = "this/file/does/not/exist.npy";

    SimulationRunFactoryImpl factory(noOpMappingAlgorithmFactory(), savingMissionControlFactory(), /*verbose=*/false);

    try {
        static_cast<void>(factory.create(simulation, minimalMissionConfig(), minimalDroneConfig(),
                                         minimalLidarConfig(), "out/simulation_run_factory_impl_test/missing_map"));
        FAIL() << "expected SimulationException for a hidden map that cannot be loaded";
    } catch (const SimulationException& e) {
        EXPECT_EQ(e.code(), "MAP_LOAD_FAILED");
    }
}

TEST(SimulationRunFactoryImpl, CreateThrowsMapLoadFailedWhenHiddenMapFileIsCorrupt) {
    const std::filesystem::path corrupt_npy = "out/simulation_run_factory_impl_test/corrupt.npy";
    std::filesystem::create_directories(corrupt_npy.parent_path());
    {
        std::ofstream corrupt_file(corrupt_npy, std::ios::binary);
        corrupt_file << "not a valid npy file";
    }

    SimulationConfigData simulation = minimalSimulationConfig();
    simulation.map_filename = corrupt_npy;

    SimulationRunFactoryImpl factory(noOpMappingAlgorithmFactory(), savingMissionControlFactory(), /*verbose=*/false);

    try {
        static_cast<void>(factory.create(simulation, minimalMissionConfig(), minimalDroneConfig(),
                                         minimalLidarConfig(), "out/simulation_run_factory_impl_test/corrupt_map"));
        FAIL() << "expected SimulationException for a corrupt hidden map file";
    } catch (const SimulationException& e) {
        EXPECT_EQ(e.code(), "MAP_LOAD_FAILED");
    }
}

// ── same failure through SimulationManager: -1/Error, no run executed, siblings unaffected ──

TEST(SimulationRunFactoryImpl, SimulationManagerScoresMissingHiddenMapNegativeOneAndDoesNotExecuteThatRunWhileSiblingRunsStillSucceed) {
    const std::filesystem::path output_path = "out/simulation_run_factory_impl_test/manager_missing_map";
    std::filesystem::remove_all(output_path);

    SimulationConfigData bad_simulation = minimalSimulationConfig();
    bad_simulation.map_filename = "this/file/does/not/exist.npy";

    SimulationCompositionData composition;
    composition.simulation_mission_groups = {
        {bad_simulation, {minimalMissionConfig()}},
        {minimalSimulationConfig(), {minimalMissionConfig()}}, // sibling simulation, loadable hidden map
    };
    composition.drone_configs = {minimalDroneConfig()};
    composition.lidar_configs = {minimalLidarConfig()};

    // Reproduces the same "sim_<i>"/"mission_<i>_<j>"/"drone_<i>"/"lidar_<i>" stems the assertions
    // below check for -- SimulationManager no longer generates these itself, so they must be
    // supplied explicitly.
    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sim_0.yaml"}, {ReferencedConfigFile{"mission_0_0.yaml"}}},
        {ReferencedConfigFile{"sim_1.yaml"}, {ReferencedConfigFile{"mission_1_0.yaml"}}},
    };
    file_paths.drone_paths = {"drone_0.yaml"};
    file_paths.lidar_paths = {"lidar_0.yaml"};

    auto factory = std::make_unique<SimulationRunFactoryImpl>(
        noOpMappingAlgorithmFactory(), savingMissionControlFactory(), /*verbose=*/false);
    SimulationManager manager(std::move(factory), file_paths);
    const SimulationManagerReport report = manager.run(composition, output_path);

    ASSERT_EQ(report.runs.size(), 2u);

    const SimulationResult& failed = report.runs[0];
    EXPECT_EQ(failed.mission_score, -1.0);
    ASSERT_EQ(failed.mission_results.size(), 1u);
    EXPECT_EQ(failed.mission_results[0].status, MissionRunStatus::Error);
    EXPECT_EQ(failed.mission_results[0].steps, 0u);
    ASSERT_EQ(failed.mission_results[0].errors.size(), 1u);
    EXPECT_EQ(failed.mission_results[0].errors[0].code, "MAP_LOAD_FAILED");
    EXPECT_FALSE(std::filesystem::exists(output_path / "simulations" / "sim_0" / "mission_0_0" / "drone_0__lidar_0" /
                                         "map_output.npy"))
        << "SimulationRunImpl::run() must never execute for a combination whose hidden map failed to load";

    const SimulationResult& sibling = report.runs[1];
    EXPECT_NE(sibling.mission_score, -1.0) << "an independent sibling simulation must still run normally";
    EXPECT_TRUE(std::filesystem::exists(output_path / "simulations" / "sim_1" / "mission_1_0" / "drone_0__lidar_0" /
                                        "map_output.npy"))
        << "the sibling simulation's run must have executed and saved its output map";
}
