// Migrated from Project 2 (FILES PROJECT 2/tests/components/simulation_output_writer_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
// namespace/include port plus one mechanical shape change forced by the new
// architecture (not a behavioral change) --
// writeSimulationOutput() dropped its separate composition_file parameter in favor of
// reading report.composition_file (a field that already existed on
// SimulationManagerReport), and CompositionFilePaths' simulation/mission path entries are
// now ReferencedConfigFile{path, load_error} instead of bare strings (§7.2) -- both
// updated in oneMissionTwoDroneFilePaths()/the test body below; no assertions changed.

#include <Simulator/SimulationOutputWriter.h>

#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;

namespace {

std::filesystem::path scratchDir() {
    const std::filesystem::path dir =
        std::filesystem::path("tests/component/test_output/simulation_output_writer_test");
    std::filesystem::create_directories(dir);
    return dir;
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

SimulationResult scoredRun(double score) {
    SimulationResult result;
    result.mission_score = score;
    result.mission_results = {MissionRunResult{MissionRunStatus::Completed, 10, {}}};
    return result;
}

SimulationResult errorRun() {
    SimulationResult result;
    result.mission_score = -1.0;
    result.mission_results = {MissionRunResult{MissionRunStatus::Error, 0, {ErrorRef{"SOME_ERROR", "boom"}}}};
    return result;
}

// One simulation, one mission, two drones x one lidar -- consumes exactly the
// two SimulationResult entries in report.runs, in order.
CompositionFilePaths oneMissionTwoDroneFilePaths() {
    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {{ReferencedConfigFile{"sim.yaml"}, {ReferencedConfigFile{"mission.yaml"}}}};
    file_paths.drone_paths = {"drone_a.yaml", "drone_b.yaml"};
    file_paths.lidar_paths = {"lidar.yaml"};
    return file_paths;
}

} // namespace

// New — closes a Project 2 mutation-coverage gap (MAN22, see
// PROJ2_TESTS_PLAN.md §8.3). SimulationOutputWriter had no component-level
// test at all before this file -- coverage-depth insurance at a fast level,
// alongside the existing
// Integration.SummaryAverageMinMaxExcludeErrorScoredRunsWhenMixedWithSuccessfulRuns
// coverage, which already catches this bug end-to-end but only via a full
// simulation binary subprocess invocation.
TEST(SimulationOutputWriter, MinScoreReflectsTrueScoredMinimumEvenWithErrorRunsPresent) {
    SimulationManagerReport report;
    report.composition_file = "composition.yaml";
    report.runs = {scoredRun(10.0), errorRun()};

    const CompositionFilePaths file_paths = oneMissionTwoDroneFilePaths();
    const std::filesystem::path output_yaml = scratchDir() / "output.yaml";

    writeSimulationOutput(report, file_paths, output_yaml);

    const YAML::Node summary = YAML::LoadFile(output_yaml.string())["score_report"]["summary"];
    ASSERT_TRUE(summary);
    EXPECT_EQ(summary["scored_runs"].as<std::size_t>(), 1u);
    EXPECT_EQ(summary["error_runs"].as<std::size_t>(), 1u);
    EXPECT_DOUBLE_EQ(summary["min_score"].as<double>(), 10.0)
        << "min_score must reflect the true minimum of the scored (non-error) runs, not be "
           "forced to -1 just because the batch also contains an error run";
}

// Locks in the output-format contract shown by the repo-root reference files
// (expected_simulation_result.yaml et al.): path/status/code strings are
// double-quoted, resolution_request_status is a bare unquoted token, and
// error_score is nested inside score_range rather than a sibling of it.
TEST(SimulationOutputWriter, StringFieldsAreDoubleQuotedAndErrorScoreNestsUnderScoreRange) {
    SimulationManagerReport report;
    report.composition_file = "composition.yaml";
    report.runs = {scoredRun(10.0), errorRun()};

    const CompositionFilePaths file_paths = oneMissionTwoDroneFilePaths();
    const std::filesystem::path output_yaml = scratchDir() / "quoting_output.yaml";

    writeSimulationOutput(report, file_paths, output_yaml);

    const std::string raw = readFile(output_yaml);
    EXPECT_NE(raw.find(R"(composition_file: "composition.yaml")"), std::string::npos);
    EXPECT_NE(raw.find(R"(drone_config: "drone_a.yaml")"), std::string::npos);
    EXPECT_NE(raw.find(R"(status: "completed")"), std::string::npos);
    EXPECT_NE(raw.find(R"(code: "SOME_ERROR")"), std::string::npos);
    EXPECT_NE(raw.find("resolution_request_status: IGNORED"), std::string::npos)
        << "resolution_request_status is an enum-like token and must stay unquoted";

    const YAML::Node score_report = YAML::LoadFile(output_yaml.string())["score_report"];
    EXPECT_FALSE(score_report["error_score"])
        << "error_score must not be a sibling of score_range";
    ASSERT_TRUE(score_report["score_range"]["error_score"])
        << "error_score must be nested inside score_range";
    EXPECT_EQ(score_report["score_range"]["error_score"].as<int>(), -1);
}

// Locks in the blank-line separators shown between run entries in
// expected_simulation_result.yaml -- including after the last run of a
// mission, which doubles as the separator before the next mission/simulation.
TEST(SimulationOutputWriter, BlankLineSeparatesEveryRunEntryIncludingTheLastOne) {
    SimulationManagerReport report;
    report.composition_file = "composition.yaml";
    report.runs = {scoredRun(10.0), errorRun()};

    const CompositionFilePaths file_paths = oneMissionTwoDroneFilePaths();
    const std::filesystem::path output_yaml = scratchDir() / "blank_line_output.yaml";

    writeSimulationOutput(report, file_paths, output_yaml);

    const std::string raw = readFile(output_yaml);
    EXPECT_NE(raw.find("score: 10\n\n            - drone_config: \"drone_b.yaml\""), std::string::npos)
        << "expected a blank line between the first and second run entries; got:\n"
        << raw;
    EXPECT_NE(raw.find("code: \"SOME_ERROR\"\n\n"), std::string::npos)
        << "expected a trailing blank line after the last run entry too; got:\n"
        << raw;
}

// Locks in the same_results/errors inline-flow, double-quoted list format
// shown in expected_comparative_report.yaml.
TEST(WriteComparativeReport, SameResultsAndErrorsAreInlineDoubleQuotedFlowLists) {
    const std::vector<ComponentRunTotals> totals = {
        {"manager1.so", 495.0, 100},
        {"manager2.so", 495.0, 100},
    };
    const std::vector<std::string> failed = {"manager3.so"};
    const std::filesystem::path output_yaml = scratchDir() / "comparative_output.yaml";

    writeComparativeReport("composition.yaml", "folder", totals, failed, output_yaml);

    const std::string raw = readFile(output_yaml);
    EXPECT_NE(raw.find(R"(same_results: ["manager1.so", "manager2.so"])"), std::string::npos);
    EXPECT_NE(raw.find(R"(errors: ["manager3.so"])"), std::string::npos);
}

// errors: [] (not YAML null/~) when nothing failed to load -- same shape as the
// populated case, just empty, so downstream YAML consumers don't need a
// separate null check.
TEST(WriteCompetitiveReport, AlgorithmFieldIsQuotedAndErrorsIsEmptyFlowListWhenNoFailures) {
    const std::vector<ComponentRunTotals> totals = {{"algorithm1.so", 495.0, 100}};
    const std::filesystem::path output_yaml = scratchDir() / "competitive_output.yaml";

    writeCompetitiveReport("composition.yaml", "mission_control.so", totals, {}, output_yaml);

    const std::string raw = readFile(output_yaml);
    EXPECT_NE(raw.find(R"(algorithm: "algorithm1.so")"), std::string::npos);
    EXPECT_NE(raw.find("errors: []"), std::string::npos);
}
