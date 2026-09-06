// First-party coverage for a fix originally surfaced by an external black-box test suite
// (ex3_135_tests, not part of this repository): PluginLoading.APluginWhoseFactoryThrowsIsSurvived.
//
// Before the fix, a MissionControl plugin that loaded/registered successfully but whose factory
// threw while constructing the instance had every one of its composition's runs converted into a
// -1 result (SimulationManager::run()'s per-run try/catch around run_factory_->create()), so the
// component finished with a complete (if all-error) SimulationManagerReport and was treated as
// having run successfully -- it never made it into the comparative report's errors: list. The fix
// (SimulationRunFactoryImpl.cpp/SimulationManager.cpp: ComponentConstructionException) lets that
// specific failure propagate out of SimulationManager::run() instead, so Simulator.cpp's existing
// runOneComponent()/aggregateOutcomes() machinery -- which already treats a thrown manager.run()
// as a failed component -- reports it correctly.
//
// Suite name `ComponentConstruction`, not `Integration.*`, mirroring the Audit suite's own
// convention of keeping author-added regression coverage out of the assignment's required
// `--gtest_filter=Integration.*` filter.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <sys/wait.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifndef DRONE_MAPPER_SIMULATION_BINARY
#define DRONE_MAPPER_SIMULATION_BINARY "./simulator_322889890_315113738"
#endif

#ifndef ALGORITHM_PLUGIN_PATH
#define ALGORITHM_PLUGIN_PATH "./Algorithm_322889890_315113738.so"
#endif

#ifndef MISSION_CONTROL_PLUGIN_PATH
#define MISSION_CONTROL_PLUGIN_PATH "./MissionControl_322889890_315113738.so"
#endif

#ifndef THROWING_MISSION_CONTROL_PLUGIN_PATH
#define THROWING_MISSION_CONTROL_PLUGIN_PATH "./throwing_mission_control_test_plugin.so"
#endif

#ifndef TEST_INPUTS_DIR
#define TEST_INPUTS_DIR "."
#endif

namespace {

std::filesystem::path testInputsDir() {
    return std::filesystem::path(TEST_INPUTS_DIR);
}

std::filesystem::path outputDir() {
    const std::filesystem::path dir = std::filesystem::path("tests/integration/test_output/component_construction");
    std::filesystem::create_directories(dir);
    return dir;
}

std::string slurpFile(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

struct RunOutcome {
    bool exited_normally = false;
    int exit_code = -1;
    std::string stderr_text;
    // The one comparative_results_<timestamp> directory the binary created directly under this
    // run's mission_control_folder, if any.
    std::filesystem::path results_dir;
};

// Spawns the real binary in -comparative mode against a mission_control_folder freshly populated
// with a copy of each {filename, source_path} pair in `plugins` -- the filename is what ends up
// under mission_control_folder (and therefore what the comparative report names in errors:/
// results_summary), independent of whatever build-system name the source .so actually has.
RunOutcome runComparativeWithPlugins(const std::string& tag, const std::filesystem::path& composition_path,
                                     const std::vector<std::pair<std::string, std::filesystem::path>>& plugins) {
    const std::filesystem::path scenario_dir = outputDir() / tag;
    std::filesystem::remove_all(scenario_dir);
    const std::filesystem::path mc_folder = scenario_dir / "mission_control_libs";
    std::filesystem::create_directories(mc_folder);
    for (const auto& [filename, source_path] : plugins) {
        std::filesystem::copy_file(source_path, mc_folder / filename);
    }

    const std::filesystem::path stderr_path = scenario_dir / "stderr.log";
    const std::filesystem::path binary_path = DRONE_MAPPER_SIMULATION_BINARY;
    const std::string command = "\"" + binary_path.string() + "\" -comparative simulation=\"" +
                                 composition_path.string() + "\" mission_control_folder=\"" + mc_folder.string() +
                                 "\" algorithm=\"" + std::string(ALGORITHM_PLUGIN_PATH) + "\" 2> \"" +
                                 stderr_path.string() + "\"";
    const int raw_status = std::system(command.c_str());

    RunOutcome outcome;
    outcome.exited_normally = WIFEXITED(raw_status);
    if (outcome.exited_normally) {
        outcome.exit_code = WEXITSTATUS(raw_status);
    }
    outcome.stderr_text = slurpFile(stderr_path);

    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(mc_folder)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("comparative_results_", 0) == 0) {
            outcome.results_dir = entry.path();
            break;
        }
    }
    return outcome;
}

std::vector<std::string> errorList(const YAML::Node& comparative_report) {
    std::vector<std::string> errors;
    for (const YAML::Node& entry : comparative_report["errors"]) {
        errors.push_back(entry.as<std::string>());
    }
    return errors;
}

bool namedInResultsSummary(const YAML::Node& comparative_report, const std::string& component_name) {
    for (const YAML::Node& group : comparative_report["results_summary"]) {
        for (const YAML::Node& name : group["same_results"]) {
            if (name.as<std::string>() == component_name) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

// A single MissionControl whose factory throws: the simulator must survive it, exit 0, and list
// the plugin's file name under the comparative report's errors: -- not merge it into
// results_summary as if it had run.
TEST(ComponentConstruction, APluginWhoseFactoryThrowsIsSurvivedAndListedInErrors) {
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_single_run.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparativeWithPlugins(
        "throwing_alone", composition_path,
        {{"throwing_mission_control.so", THROWING_MISSION_CONTROL_PLUGIN_PATH}});

    ASSERT_TRUE(run.exited_normally) << "a throwing plugin must not take the simulator down; stderr:\n"
                                     << run.stderr_text;
    EXPECT_EQ(run.exit_code, 0) << "stderr was:\n" << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "expected a comparative_results_* directory; stderr was:\n"
                                          << run.stderr_text;

    const std::filesystem::path report_path = run.results_dir / "comparative_report.yaml";
    ASSERT_TRUE(std::filesystem::exists(report_path));
    const YAML::Node report = YAML::LoadFile(report_path.string())["comparative_report"];
    ASSERT_TRUE(report);

    const std::vector<std::string> errors = errorList(report);
    EXPECT_NE(std::find(errors.begin(), errors.end(), "throwing_mission_control.so"), errors.end())
        << "a plugin whose factory throws must be listed under errors:";
    EXPECT_FALSE(namedInResultsSummary(report, "throwing_mission_control.so"))
        << "a component that never produced a real result must not appear in results_summary";
}

// A throwing plugin alongside a good one: the good plugin must still run and be ranked; only the
// throwing one is reported as failed.
TEST(ComponentConstruction, GoodPluginStillRunsAlongsideAThrowingOneAndOnlyTheThrowingOneIsAnError) {
    const std::filesystem::path composition_path = testInputsDir() / "compositions" / "composition_single_run.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparativeWithPlugins(
        "throwing_alongside_good", composition_path,
        {{"good_mission_control.so", MISSION_CONTROL_PLUGIN_PATH},
         {"throwing_mission_control.so", THROWING_MISSION_CONTROL_PLUGIN_PATH}});

    ASSERT_TRUE(run.exited_normally) << run.stderr_text;
    EXPECT_EQ(run.exit_code, 0) << "stderr was:\n" << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr was:\n" << run.stderr_text;

    const std::filesystem::path report_path = run.results_dir / "comparative_report.yaml";
    ASSERT_TRUE(std::filesystem::exists(report_path));
    const YAML::Node report = YAML::LoadFile(report_path.string())["comparative_report"];
    ASSERT_TRUE(report);

    const std::vector<std::string> errors = errorList(report);
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors.front(), "throwing_mission_control.so");

    EXPECT_TRUE(namedInResultsSummary(report, "good_mission_control.so"))
        << "the good plugin must still run and appear in results_summary despite a sibling "
           "plugin's factory throwing";

    const std::filesystem::path good_output_yaml = run.results_dir / "simulation_output_good_mission_control.yaml";
    EXPECT_TRUE(std::filesystem::exists(good_output_yaml))
        << "the good plugin's per-component YAML must still be written";
}

// The semantic constraint this fix must not violate: a component whose composition legitimately
// contains a -1 run (a real per-run failure, not a factory construction failure) must still be
// reported as having run -- present in results_summary, absent from errors:.
TEST(ComponentConstruction, AComponentWithLegitimateRunLevelErrorsIsNotClassifiedAsAFailedComponent) {
    const std::filesystem::path composition_path =
        testInputsDir() / "compositions" / "composition_mixed_success_and_error.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparativeWithPlugins(
        "mixed_run_level_errors", composition_path,
        {{"good_mission_control.so", MISSION_CONTROL_PLUGIN_PATH}});

    ASSERT_TRUE(run.exited_normally) << run.stderr_text;
    EXPECT_EQ(run.exit_code, 0) << "stderr was:\n" << run.stderr_text;
    ASSERT_FALSE(run.results_dir.empty()) << "stderr was:\n" << run.stderr_text;

    const std::filesystem::path output_yaml = run.results_dir / "simulation_output_good_mission_control.yaml";
    ASSERT_TRUE(std::filesystem::exists(output_yaml))
        << "a component with some -1 runs must still produce its per-component YAML";
    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report);
    EXPECT_GT(score_report["summary"]["error_runs"].as<std::size_t>(), 0u)
        << "sanity check: this composition must actually contain at least one run-level failure";

    const std::filesystem::path report_path = run.results_dir / "comparative_report.yaml";
    ASSERT_TRUE(std::filesystem::exists(report_path));
    const YAML::Node report = YAML::LoadFile(report_path.string())["comparative_report"];
    ASSERT_TRUE(report);

    EXPECT_TRUE(errorList(report).empty())
        << "a component with only ordinary per-run -1 scores must not appear in errors:";
    EXPECT_TRUE(namedInResultsSummary(report, "good_mission_control.so"))
        << "a component with only ordinary per-run -1 scores must still be ranked in results_summary";
}
