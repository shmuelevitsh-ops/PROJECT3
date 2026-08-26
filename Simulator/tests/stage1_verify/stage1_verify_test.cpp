// Permanent coverage for Stage 1 of SIMULATOR_CORE_PLAN.md, merged into
// simulator_registration_test -- exercises SimulationRunFactoryImpl
// end-to-end through SimulationManager on a single simulation/mission/drone/lidar
// combination, using dynamically-loaded plugin factories the same way
// registration_end_to_end_test does. Isolated to one combo (rather than the
// full composition, which takes several minutes and previously crashed the
// devcontainer) to keep this fast. small_simulation_out/small_mission_out is
// used because its mission boundaries overlap the origin -- unlike the
// *_room missions, which fail the boundary-overlap check instantly and
// always score -1, never exercising the mapping algorithm / mission control.

#include <Simulator/ConfigLoader.h>
#include <Simulator/Registrar.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

TEST(Stage1Verify, RunsSingleCombo) {
    auto& registrar = simulator::Registrar::instance();

    // Registrar only reruns a plugin's REGISTER_* constructor on a genuinely fresh dlopen path
    // (same literal path a second time throws PLUGIN_NOT_REGISTERED). Since this test now shares a
    // process with other suites that may already load ALGORITHM_PLUGIN_PATH/MISSION_CONTROL_PLUGIN_PATH,
    // copy each plugin into its own uniquely-named scratch file first, so this load is unambiguously
    // this process's first-ever load of that exact path.
    const std::filesystem::path scratch_dir =
        std::filesystem::temp_directory_path() / "simulator_stage1_verify_plugins";
    std::filesystem::remove_all(scratch_dir);
    std::filesystem::create_directories(scratch_dir);
    const std::filesystem::path algorithm_copy =
        scratch_dir / std::filesystem::path(ALGORITHM_PLUGIN_PATH).filename();
    const std::filesystem::path mission_control_copy =
        scratch_dir / std::filesystem::path(MISSION_CONTROL_PLUGIN_PATH).filename();
    std::filesystem::copy_file(ALGORITHM_PLUGIN_PATH, algorithm_copy);
    std::filesystem::copy_file(MISSION_CONTROL_PLUGIN_PATH, mission_control_copy);

    const common::MappingAlgorithmFactory mapping_algorithm_factory =
        registrar.loadMappingAlgorithm(algorithm_copy);
    ASSERT_TRUE(mapping_algorithm_factory);
    const common::MissionControlFactory mission_control_factory =
        registrar.loadMissionControl(mission_control_copy);
    ASSERT_TRUE(mission_control_factory);

    const std::filesystem::path inputs = TEST_INPUTS_DIR;
    const std::filesystem::path simulation_path = inputs / "simulation/small_simulation_out_e2e.yaml";
    const std::filesystem::path mission_path = inputs / "mission/small_mission_out.yaml";
    const std::filesystem::path drone_path = inputs / "drone/drone_small.yaml";
    const std::filesystem::path lidar_path = inputs / "lidar/lidar_short.yaml";

    const simulator::types::SimulationConfigData simulation = simulator::parseSimulationConfig(simulation_path);
    const common::types::MissionConfigData mission = simulator::parseMissionConfig(mission_path);
    const common::types::DroneConfigData drone = simulator::parseDroneConfig(drone_path);
    const common::types::LidarConfigData lidar = simulator::parseLidarConfig(lidar_path);

    simulator::types::SimulationCompositionData composition;
    composition.simulation_mission_groups.push_back({simulation, {mission}});
    composition.drone_configs.push_back(drone);
    composition.lidar_configs.push_back(lidar);

    // The real config paths behind the parsed structs above, matching composition's shape exactly.
    simulator::CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths.push_back(
        {simulator::ReferencedConfigFile{simulation_path.string()},
         {simulator::ReferencedConfigFile{mission_path.string()}}});
    file_paths.drone_paths.push_back(drone_path.string());
    file_paths.lidar_paths.push_back(lidar_path.string());

    auto factory_impl = std::make_unique<simulator::SimulationRunFactoryImpl>(
        mapping_algorithm_factory, mission_control_factory, /*verbose=*/false);
    simulator::SimulationManager manager{std::move(factory_impl), file_paths};

    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "simulator_stage1_verify_single";
    std::filesystem::remove_all(output_dir);
    std::filesystem::create_directories(output_dir);

    simulator::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(composition, output_dir));

    ASSERT_EQ(report.runs.size(), 1u);
    const auto& run = report.runs.front();
    std::cerr << "mission_score=" << run.mission_score
              << " output_map_file=" << run.output_map_file
              << " exists=" << std::filesystem::exists(run.output_map_file) << '\n';
    EXPECT_TRUE(std::filesystem::exists(run.output_map_file));
    EXPECT_NE(run.mission_score, -1.0);
}
