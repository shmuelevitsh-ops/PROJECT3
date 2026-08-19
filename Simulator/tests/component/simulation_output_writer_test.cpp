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
