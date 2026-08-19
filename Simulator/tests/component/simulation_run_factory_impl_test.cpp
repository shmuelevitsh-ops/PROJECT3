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

#include <Simulator/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;

namespace {

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

// Minimal-but-valid configs: a nonexistent map_filename takes the documented graceful-fallback
// path (logs to std::cerr, builds an empty/Unmapped hidden map sized from the still-positive
// map_resolution -- see SimulationRunFactoryImpl.cpp's loadHiddenMap()). max_steps = 0 means
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
    config.map_filename = "this/file/does/not/exist.npy";
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
