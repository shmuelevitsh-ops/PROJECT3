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

// Reads a file's raw bytes -- needed for the formatting-only assertions below (quoting, flow-vs-
// block sequence style, trailing newline), none of which survive a YAML::LoadFile() round trip:
// a parser discards the distinction between "foo" and foo, and between inline and block lists.
std::string readRaw(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
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

// ── comparative/competitive report formatting ───────────────────────────────
// The staff format requires double-quoted strings, inline (flow-style) .so lists, `errors: []`
// instead of a YAML null, and a trailing newline. None of that survives a YAML::LoadFile() round
// trip (a parser normalizes away quoting and flow-vs-block style, and treats `[]` and `~` as
// merely "empty" vs "null" -- both still falsy-ish), so these tests check the raw file bytes for
// the formatting-specific assertions, alongside a YAML::LoadFile() parse to confirm the same
// fields/values (composition_file, mission_control_folder, same_results, total_score,
// total_steps) are still present and correct -- i.e. only the serialization changed, not the
// report's actual content.

TEST(SimulationOutputWriter, ComparativeReportQuotesStringsAndInlinesSameResults) {
    const std::vector<ComponentRunTotals> totals = {
        ComponentRunTotals{"manager_a.so", 100.0, 50},
        ComponentRunTotals{"manager_b.so", 100.0, 50},
    };
    const std::filesystem::path output_yaml = scratchDir() / "comparative_quoting.yaml";

    writeComparativeReport("inputs/composition.yaml", "mission_controls", totals, {}, output_yaml);

    const std::string raw = readRaw(output_yaml);
    EXPECT_NE(raw.find(R"(composition_file: "inputs/composition.yaml")"), std::string::npos)
        << "composition_file must be double-quoted, got:\n" << raw;
    EXPECT_NE(raw.find(R"(mission_control_folder: "mission_controls")"), std::string::npos)
        << "mission_control_folder must be double-quoted, got:\n" << raw;
    EXPECT_NE(raw.find(R"(same_results: ["manager_a.so", "manager_b.so"])"), std::string::npos)
        << "same_results must be an inline (flow-style) list of double-quoted .so names, got:\n" << raw;

    const YAML::Node parsed = YAML::LoadFile(output_yaml.string())["comparative_report"];
    ASSERT_TRUE(parsed);
    EXPECT_EQ(parsed["composition_file"].as<std::string>(), "inputs/composition.yaml")
        << "quoting must not change the actual parsed value";
    const YAML::Node group = parsed["results_summary"][0];
    EXPECT_EQ(group["same_results"][0].as<std::string>(), "manager_a.so");
    EXPECT_EQ(group["same_results"][1].as<std::string>(), "manager_b.so");
    EXPECT_DOUBLE_EQ(group["total_score"].as<double>(), 100.0);
    EXPECT_EQ(group["total_steps"].as<std::size_t>(), 50u);
}

TEST(SimulationOutputWriter, ComparativeReportEmitsEmptyErrorsAsInlineListNotNull) {
    const std::vector<ComponentRunTotals> totals = {ComponentRunTotals{"manager.so", 1.0, 1}};
    const std::filesystem::path output_yaml = scratchDir() / "comparative_empty_errors.yaml";

    writeComparativeReport("composition.yaml", "mission_controls", totals, {}, output_yaml);

    const std::string raw = readRaw(output_yaml);
    EXPECT_NE(raw.find("errors: []"), std::string::npos)
        << "an empty errors list must be emitted as an inline empty list, not YAML null, got:\n" << raw;
    EXPECT_EQ(raw.find("errors: ~"), std::string::npos);

    const YAML::Node errors = YAML::LoadFile(output_yaml.string())["comparative_report"]["errors"];
    ASSERT_TRUE(errors);
    EXPECT_TRUE(errors.IsSequence()) << "errors must parse back as an (empty) sequence, not null";
    EXPECT_EQ(errors.size(), 0u);
}

TEST(SimulationOutputWriter, ComparativeReportNonEmptyErrorsAreQuotedAndInline) {
    const std::vector<ComponentRunTotals> totals = {ComponentRunTotals{"manager.so", 1.0, 1}};
    const std::filesystem::path output_yaml = scratchDir() / "comparative_nonempty_errors.yaml";

    writeComparativeReport("composition.yaml", "mission_controls", totals,
                           {"broken_a.so", "broken_b.so"}, output_yaml);

    const std::string raw = readRaw(output_yaml);
    EXPECT_NE(raw.find(R"(errors: ["broken_a.so", "broken_b.so"])"), std::string::npos)
        << "non-empty errors must be an inline (flow-style) list of double-quoted .so names, got:\n" << raw;
}

TEST(SimulationOutputWriter, ComparativeReportFileEndsWithExactlyOneTrailingNewline) {
    const std::vector<ComponentRunTotals> totals = {ComponentRunTotals{"manager.so", 1.0, 1}};
    const std::filesystem::path output_yaml = scratchDir() / "comparative_trailing_newline.yaml";

    writeComparativeReport("composition.yaml", "mission_controls", totals, {}, output_yaml);

    const std::string raw = readRaw(output_yaml);
    ASSERT_FALSE(raw.empty());
    EXPECT_EQ(raw.back(), '\n') << "file must end with a newline";
    EXPECT_NE(raw[raw.size() - 2], '\n') << "file must not end with a blank line";
}

TEST(SimulationOutputWriter, CompetitiveReportQuotesStringsIncludingMissionControlAndAlgorithm) {
    const std::vector<ComponentRunTotals> totals = {
        ComponentRunTotals{"algorithm_a.so", 20.0, 5},
        ComponentRunTotals{"algorithm_b.so", 10.0, 3},
    };
    const std::filesystem::path output_yaml = scratchDir() / "competitive_quoting.yaml";

    writeCompetitiveReport("inputs/composition.yaml", "manager.so", totals, {}, output_yaml);

    const std::string raw = readRaw(output_yaml);
    EXPECT_NE(raw.find(R"(mission_control: "manager.so")"), std::string::npos)
        << "mission_control must be double-quoted, got:\n" << raw;
    EXPECT_NE(raw.find(R"(algorithm: "algorithm_a.so")"), std::string::npos)
        << "algorithm must be double-quoted, got:\n" << raw;
    EXPECT_NE(raw.find("errors: []"), std::string::npos);
    ASSERT_FALSE(raw.empty());
    EXPECT_EQ(raw.back(), '\n') << "file must end with a newline";

    const YAML::Node parsed = YAML::LoadFile(output_yaml.string())["competitive_report"];
    ASSERT_TRUE(parsed);
    EXPECT_EQ(parsed["results_summary"][0]["algorithm"].as<std::string>(), "algorithm_a.so")
        << "quoting must not change the actual parsed value or its rank ordering";
}
