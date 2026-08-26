// Migrated from Project 2 (FILES PROJECT 2/tests/audit/audit_errors_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4.
//
// NON-MECHANICAL ADAPTATION beyond namespaces/binary name/fixture paths: Project 3's
// Simulator CLI is not Project 2's `drone_mapper_simulation <composition> <output_dir>`
// contract. It is `-comparative simulation=<file> mission_control_folder=<dir>
// algorithm=<file>` (or the symmetric `-competition`), where `mission_control_folder`
// is scanned for candidate .so plugins and the results directory
// (`comparative_results_<timestamp>`) is auto-created by the binary itself directly
// under that folder — there is no caller-specified output path at all (see
// CliOptions.cpp/Simulator.cpp, built in an earlier Project 3 stage per
// SIMULATOR_CORE_PLAN.md). This was only discovered while running these tests here,
// since no earlier migration phase ever spawned the actual compiled binary. Every test
// below now runs -comparative mode against a scratch mission_control_folder containing
// exactly one copy of the real, freshly-built MissionControl_322889890_315113738.so (so
// each run produces exactly one component's worth of output, preserving this file's
// original "one composition run, one result" shape) and locates the created
// comparative_results_* directory to find error.log / simulation_output_<stem>.yaml.
// SimulationManager/ConfigLoader's own per-run isolation behavior — the actual thing
// every scenario below protects — is completely unchanged; only where its output lands
// moved. writeSimulationOutput()'s YAML schema is documented unchanged from Assignment 2
// (SimulationOutputWriter.h), so every score_report/summary/runs assertion below is
// unchanged from the Project 2 original.
//
// Per-test disposition (PROJ2_TESTS_PLAN.md §5.1, confirmed baseline 3/5 pass
// in Project 2):
//   - MissingCompositionFileStillHaltsWithNoPerRunIsolationPossible: intent preserved,
//     but its observable signal changed along with the CLI (see the test's own comment
//     below) — Project 3's CliOptions validates `simulation=<file>` for existence
//     *before* any results directory is created at all, so this is now a clean CLI
//     usage error (stderr, exit 1, nothing created) rather than a later parse-time
//     exception caught by main()'s top-level handler and logged to an error.log that,
//     in this scenario, never gets created.
//   - MissionBoundaryInvalidIsIsolatedToItsOwnErrorEntry: near-1:1 (mechanical CLI/path
//     adaptation only).
//   - GroupLevelFailureFillsMinusOneForEveryMissionInTheGroupWhileSiblingGroupScoresNormally:
//     near-1:1 (mechanical CLI/path adaptation only).
//   - NonPositiveResolutionIsIsolatedToItsOwnErrorEntry: NOT migrated. Stale —
//     assumes mission.gps_resolution_cm: 0 reaches Map3DImpl as the actual map
//     resolution used to build the output map; the implementation instead uses
//     the simulation's own map resolution for that purpose, so the scenario
//     this test is built around cannot occur. Project 2's implementation was
//     not modified to make this test pass — the test's premise is outdated,
//     not the code.
//   - MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally: structurally
//     adapted. Valuable intent preserved (one bad run in a batch isolated/
//     scored -1 while a sibling good run in the same batch still scores
//     normally), but its second "bad" run used to reuse the same stale
//     invalid-resolution scenario above. Rebuilt around a genuinely reachable
//     error case instead — a mission_config reference that does not exist,
//     which ConfigLoader::parseMissionOrPlaceholder() surfaces as a per-run
//     MISSION_CONFIG_LOAD_FAILED load_error, distinct from the sibling
//     MISSION_BOUNDARY_INVALID bad run already covered above. See
//     composition_mixed_pass_and_fail.yaml's own comment for detail. (Plus the
//     same CLI/path adaptation as the other three migrated tests.)

// AUDIT SUITE — diagnostic-only, for local inspection of error.log /
// simulation_output_<component>.yaml behavior on unrecoverable per-run initialization
// errors, after the SimulationManager per-run isolation fix.
//
// Suite name is `Audit`, not `Integration`, so it is never picked up by the
// assignment's required `--gtest_filter=Integration.*` filter.
//
// These tests verify per-run isolation: a failing run is isolated to its own
// entry (status: "error", score: -1, error_ref.code set), the rest of the
// composition's runs are scored normally, and the binary still exits 0 with a
// complete per-component YAML — matching the assignment's documented Error
// Handling Policy ("the failed scenario should get the score -1 ... the
// simulation should continue to the next scenario").
//
// The one scenario that is NOT per-run-isolated, and is not supposed to be,
// is a missing/unusable top-level composition file: SimulationManager never
// sees any runs at all, so there is no per-run list to isolate a failure
// within.
//
// ConfigLoader.cpp does not drop an entire simulation_config group (or a
// single bad mission_config) when its file fails to load -- it fills a
// load_error placeholder instead, and SimulationManager checks that
// placeholder before ever calling the run factory. The group-level scenario
// below demonstrates this: a broken simulation_config fills -1 for every
// mission declared under it, while a sibling group in the same composition is
// entirely unaffected.
//
// No drone-movement/collision scenario is used anywhere in this file, per
// the audit's explicit constraint.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <sys/wait.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#ifndef AUDIT_INPUTS_DIR
#define AUDIT_INPUTS_DIR "."
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

namespace {

std::filesystem::path auditInputsDir() {
    return std::filesystem::path(AUDIT_INPUTS_DIR);
}

std::filesystem::path auditOutputDir() {
    const std::filesystem::path dir = std::filesystem::path("tests/audit/audit_output");
    std::filesystem::create_directories(dir);
    return dir;
}

std::string slurpFile(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

// The stem shared by every scenario's single candidate component -- always a copy of the same
// real MissionControl_322889890_315113738.so (see runComparative() below), so every
// per-component YAML this file inspects is always named "simulation_output_<this>.yaml".
std::string missionControlStem() {
    return std::filesystem::path(MISSION_CONTROL_PLUGIN_PATH).stem().string();
}

struct RunOutcome {
    bool exited_normally = false;
    int exit_code = -1;
    std::string stderr_text;
    // The one comparative_results_<timestamp> directory the binary created directly under this
    // run's mission_control_folder, if any -- empty if the run never got that far (e.g. a CLI
    // validation failure).
    std::filesystem::path results_dir;
};

// Spawns the real binary in -comparative mode against `composition_path`, using a dedicated,
// freshly-cleared mission_control_folder (under tests/audit/audit_output/<tag>/) containing
// exactly one candidate .so -- a copy of the real, freshly-built MissionControl plugin -- so this
// run always produces exactly one component's worth of output, matching this file's original
// "one composition run, one result" scenarios. See the file banner comment for why this replaced
// Project 2's simple <composition> <output_dir> CLI.
RunOutcome runComparative(const std::string& tag, const std::filesystem::path& composition_path) {
    const std::filesystem::path scenario_dir = auditOutputDir() / tag;
    std::filesystem::remove_all(scenario_dir);
    const std::filesystem::path mc_folder = scenario_dir / "mission_control_libs";
    std::filesystem::create_directories(mc_folder);
    const std::filesystem::path plugin = MISSION_CONTROL_PLUGIN_PATH;
    std::filesystem::copy_file(plugin, mc_folder / plugin.filename());

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

} // namespace

// ── Scenario 1: missing/unusable top-level composition file ───────────────
//
// simulation=<path> points at a path that does not exist. Project 3's CliOptions validates every
// mandatory file/folder argument for existence up front (CliOptions.cpp's isOpenableFile()),
// *before* main() ever creates a results directory or opens an error.log -- so there is no
// per-run list, and not even an error.log, to isolate a failure within. Confirms the fix is
// correctly scoped: it isolates failures *within* a parsed composition's runs, not a failure to
// even begin one.
TEST(Audit, MissingCompositionFileStillHaltsWithNoPerRunIsolationPossible) {
    const std::filesystem::path missing_composition =
        auditInputsDir() / "compositions" / "this_file_does_not_exist.yaml";
    ASSERT_FALSE(std::filesystem::exists(missing_composition)) << "fixture sanity check";

    const RunOutcome run = runComparative("missing_composition_file", missing_composition);

    ASSERT_TRUE(run.exited_normally) << "main() must finish normally, never via exit()/abort()";
    EXPECT_EQ(run.exit_code, 1);
    EXPECT_NE(run.stderr_text.find("is not an existing, openable file"), std::string::npos)
        << "stderr was:\n" << run.stderr_text;

    EXPECT_TRUE(run.results_dir.empty())
        << "no composition was ever parsed -- there is no run list to isolate a failure within, so "
           "no results directory should have been created at all; found: " << run.results_dir;
}

// ── Scenario 2: MISSION_BOUNDARY_INVALID, isolated to its own run ────────
//
// mission_invalid_boundary.yaml's x_boundary has min_cm (140) > max_cm (50).
// SimulationRunFactoryImpl::outputMapConfig() -> Map3DImpl's constructor
// throws simulator::SimulationException("MISSION_BOUNDARY_INVALID", ...)
// (src/Map3DImpl.cpp). SimulationManager::run() catches this
// per-run (src/SimulationManager.cpp), so the binary exits 0, writes a
// complete per-component YAML, and this run's own entry carries
// status: "error" / score: -1 / error_ref.code: "MISSION_BOUNDARY_INVALID".
TEST(Audit, MissionBoundaryInvalidIsIsolatedToItsOwnErrorEntry) {
    const std::filesystem::path composition_path =
        auditInputsDir() / "compositions" / "composition_invalid_boundary.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("mission_boundary_invalid", composition_path);

    ASSERT_TRUE(run.exited_normally);
    EXPECT_EQ(run.exit_code, 0) << "a per-run init failure must no longer abort the whole binary";
    ASSERT_FALSE(run.results_dir.empty()) << "expected a comparative_results_* directory; stderr was:\n"
                                          << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    ASSERT_TRUE(std::filesystem::exists(error_log));
    const std::string log_contents = slurpFile(error_log);
    EXPECT_NE(log_contents.find("invalid mapping boundaries"), std::string::npos) << "error.log was:\n" << log_contents;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml)) << "the failed run must not prevent the YAML from being written";

    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report);
    EXPECT_EQ(score_report["summary"]["total_runs"].as<std::size_t>(), 1u);
    EXPECT_EQ(score_report["summary"]["error_runs"].as<std::size_t>(), 1u);
    EXPECT_EQ(score_report["summary"]["scored_runs"].as<std::size_t>(), 0u);

    const YAML::Node run_node = score_report["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run_node["status"].as<std::string>(), "error");
    EXPECT_EQ(run_node["score"].as<double>(), -1.0);
    EXPECT_EQ(run_node["error_ref"]["code"].as<std::string>(), "MISSION_BOUNDARY_INVALID");
}

// ── Scenario 3 (Step 3): group-level failure -- a bad simulation_config ──
//
// composition_group_failure.yaml's first group references
// sim_config_missing_resolution.yaml, which is missing map_resolution_cm
// entirely -- ConfigLoader::parseSimulationConfig() throws when reading
// that field. ConfigLoader.cpp's placeholder-filling behavior means this is
// NOT silently dropped: every mission declared under that broken
// simulation_config (mission_a, mission_b -- both perfectly valid missions
// in their own right) still gets an entry in the per-component YAML, each
// with its own status: "error" / score: -1 / error_ref.code:
// "SIMULATION_CONFIG_LOAD_FAILED", set directly by SimulationManager
// checking SimulationConfigData::load_error before ever calling the run
// factory. The second, sibling group (sim_2 + mission_c) is a real, working
// group entirely unaffected by the first group's failure -- proving the fix
// isolates at the group level without disturbing anything else in the same
// composition.
TEST(Audit, GroupLevelFailureFillsMinusOneForEveryMissionInTheGroupWhileSiblingGroupScoresNormally) {
    const std::filesystem::path composition_path =
        auditInputsDir() / "compositions" / "composition_group_failure.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("group_level_failure", composition_path);

    ASSERT_TRUE(run.exited_normally);
    EXPECT_EQ(run.exit_code, 0) << "a broken simulation_config in one group must not abort the whole binary";
    ASSERT_FALSE(run.results_dir.empty()) << "expected a comparative_results_* directory; stderr was:\n"
                                          << run.stderr_text;

    const std::filesystem::path error_log = run.results_dir / "error.log";
    ASSERT_TRUE(std::filesystem::exists(error_log));
    const std::string log_contents = slurpFile(error_log);
    EXPECT_NE(log_contents.find("simulation_config failed to load"), std::string::npos)
        << "error.log was:\n" << log_contents;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml));

    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report);

    // Group 1 (broken sim config): mission_a + mission_b, 1 drone x 1 lidar == 2 runs, both -1.
    // Group 2 (sim_2, working): mission_c, 1 drone x 1 lidar == 1 run, scored normally.
    const YAML::Node summary = score_report["summary"];
    EXPECT_EQ(summary["total_runs"].as<std::size_t>(), 3u);
    EXPECT_EQ(summary["scored_runs"].as<std::size_t>(), 1u);
    EXPECT_EQ(summary["error_runs"].as<std::size_t>(), 2u);

    const YAML::Node simulations = score_report["simulations"];
    ASSERT_EQ(simulations.size(), 2u);

    // Group 1: every mission under the broken simulation_config is -1, regardless of mission
    // validity -- mission_a and mission_b are themselves perfectly valid fixtures.
    const YAML::Node broken_group_missions = simulations[0]["missions"];
    ASSERT_EQ(broken_group_missions.size(), 2u);
    for (const YAML::Node& mission_node : broken_group_missions) {
        const YAML::Node failed_run = mission_node["runs"][0];
        EXPECT_EQ(failed_run["status"].as<std::string>(), "error");
        EXPECT_EQ(failed_run["score"].as<double>(), -1.0);
        EXPECT_EQ(failed_run["error_ref"]["code"].as<std::string>(), "SIMULATION_CONFIG_LOAD_FAILED");
    }

    // Group 2 (sim_2 + mission_c): a real, working sibling group, entirely unaffected by group
    // 1's failure -- scored normally, never "error".
    const YAML::Node sibling_group_missions = simulations[1]["missions"];
    ASSERT_EQ(sibling_group_missions.size(), 1u);
    const YAML::Node good_run = sibling_group_missions[0]["runs"][0];
    const std::string good_status = good_run["status"].as<std::string>();
    EXPECT_TRUE(good_status == "completed" || good_status == "max_steps") << "got status: " << good_status;
    const double good_score = good_run["score"].as<double>();
    EXPECT_GE(good_score, 0.0);
    EXPECT_LE(good_score, 100.0);
}

// ── Scenario 4: mixed batch -- exactly the assignment's documented shape ──
//
// One simulation, three missions under it: a real, valid region
// (mission_a.yaml, reused verbatim from tests/integration/test_inputs/) plus
// two broken missions -- mission_invalid_boundary.yaml
// (MISSION_BOUNDARY_INVALID) and a reference to a mission_config file that
// does not exist (MISSION_CONFIG_LOAD_FAILED; see
// composition_mixed_pass_and_fail.yaml's own comment for why this replaced
// the original invalid-resolution fixture). Demonstrates the actual point of
// the fix: in a single composition with both good and bad runs, the bad ones
// are marked error/-1 individually while the good one is scored normally --
// the same shape as the assignment spec's own worked example (12 runs,
// 2 errors, 10 scored).
TEST(Audit, MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally) {
    const std::filesystem::path composition_path =
        auditInputsDir() / "compositions" / "composition_mixed_pass_and_fail.yaml";
    ASSERT_TRUE(std::filesystem::exists(composition_path)) << composition_path;

    const RunOutcome run = runComparative("mixed_pass_and_fail", composition_path);

    ASSERT_TRUE(run.exited_normally);
    EXPECT_EQ(run.exit_code, 0) << "a mixed batch with some bad runs must still exit 0";
    ASSERT_FALSE(run.results_dir.empty()) << "expected a comparative_results_* directory; stderr was:\n"
                                          << run.stderr_text;

    const std::filesystem::path output_yaml = run.results_dir / ("simulation_output_" + missionControlStem() + ".yaml");
    ASSERT_TRUE(std::filesystem::exists(output_yaml));

    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    ASSERT_TRUE(score_report);

    // 1 simulation x 3 missions x 1 drone x 1 lidar == 3 runs: mission_a (good),
    // mission_invalid_boundary (bad), the missing mission_config reference (bad).
    const YAML::Node summary = score_report["summary"];
    EXPECT_EQ(summary["total_runs"].as<std::size_t>(), 3u);
    EXPECT_EQ(summary["scored_runs"].as<std::size_t>(), 1u);
    EXPECT_EQ(summary["error_runs"].as<std::size_t>(), 2u);

    const YAML::Node missions = score_report["simulations"][0]["missions"];
    ASSERT_EQ(missions.size(), 3u);

    // mission_a: real region, real score, "completed" or "max_steps" -- never "error".
    const std::string good_mission_config = missions[0]["mission_config"].as<std::string>();
    EXPECT_NE(good_mission_config.find("mission_a"), std::string::npos);
    const YAML::Node good_run = missions[0]["runs"][0];
    const std::string good_status = good_run["status"].as<std::string>();
    EXPECT_TRUE(good_status == "completed" || good_status == "max_steps") << "got status: " << good_status;
    EXPECT_GE(good_run["score"].as<double>(), 0.0);
    EXPECT_LE(good_run["score"].as<double>(), 100.0);

    // mission_invalid_boundary: isolated error, doesn't affect mission_a above or the
    // missing mission_config reference below.
    const YAML::Node boundary_run = missions[1]["runs"][0];
    EXPECT_EQ(boundary_run["status"].as<std::string>(), "error");
    EXPECT_EQ(boundary_run["score"].as<double>(), -1.0);
    EXPECT_EQ(boundary_run["error_ref"]["code"].as<std::string>(), "MISSION_BOUNDARY_INVALID");

    // The missing mission_config reference: isolated error, independent of the other two.
    const YAML::Node missing_config_run = missions[2]["runs"][0];
    EXPECT_EQ(missing_config_run["status"].as<std::string>(), "error");
    EXPECT_EQ(missing_config_run["score"].as<double>(), -1.0);
    EXPECT_EQ(missing_config_run["error_ref"]["code"].as<std::string>(), "MISSION_CONFIG_LOAD_FAILED");
}
