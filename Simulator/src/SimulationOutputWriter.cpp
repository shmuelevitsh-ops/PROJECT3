#include <Simulator/SimulationOutputWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

namespace simulator {

namespace common_types = common::types;

using common::cm;

namespace {

std::string currentUtcTimestamp() {
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc_tm{};
    gmtime_r(&now_time_t, &utc_tm);
    std::ostringstream out;
    out << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

std::string statusToString(common_types::MissionRunStatus status) {
    switch (status) {
    case common_types::MissionRunStatus::Completed:
        return "completed";
    case common_types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case common_types::MissionRunStatus::Error:
        return "error";
    }
    return "error";
}

std::string resolutionStatusToString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED TOO SMALL";
    }
    return "IGNORED"; // unreachable for a valid enum value
}

// One run: { drone_config, lidar_config, status, steps, score, error_ref? }.
// error_ref is printed whenever errors is non-empty, regardless of status —
// preserves diagnostics even for a "completed" run with a recorded warning
// such as unmappable voxels, not just for status == "error".
YAML::Node buildRunNode(const types::SimulationResult& result, const std::string& drone_path,
                        const std::string& lidar_path) {
    const common_types::MissionRunResult& mission_result = result.mission_results.at(0);

    YAML::Node node;
    node["drone_config"] = drone_path;
    node["lidar_config"] = lidar_path;
    node["status"] = statusToString(mission_result.status);
    node["steps"] = mission_result.steps;
    node["score"] = result.mission_score;

    if (!mission_result.errors.empty()) {
        YAML::Node error_node;
        error_node["code"] = mission_result.errors.front().code;
        node["error_ref"] = error_node;
    }
    return node;
}

// One mission: { mission_config, resolution_cm, resolution_request_status, runs }.
// Consumes exactly drone_paths.size() * lidar_paths.size() entries from
// `runs`, starting at `run_index` (advanced by reference), matching
// SimulationManager::run()'s nested-loop order. resolution_cm and
// resolution_request_status are the same across every run in one mission
// (both decided once per mission, independent of drone/lidar), so they're
// read off the first consumed run.
YAML::Node buildMissionNode(const std::string& mission_path, const std::vector<types::SimulationResult>& runs,
                           std::size_t& run_index, const std::vector<std::string>& drone_paths,
                           const std::vector<std::string>& lidar_paths) {
    YAML::Node mission_node;
    mission_node["mission_config"] = mission_path;

    YAML::Node runs_node;
    bool first_run = true;
    for (const std::string& drone_path : drone_paths) {
        for (const std::string& lidar_path : lidar_paths) {
            const types::SimulationResult& result = runs.at(run_index++);
            if (first_run) {
                mission_node["resolution_cm"] = result.output_map_config.resolution.force_numerical_value_in(cm);
                mission_node["resolution_request_status"] = resolutionStatusToString(result.resolution_request_status);
                first_run = false;
            }
            runs_node.push_back(buildRunNode(result, drone_path, lidar_path));
        }
    }
    mission_node["runs"] = runs_node;
    return mission_node;
}

// The `simulations:` hierarchy: walks file_paths.simulation_mission_paths
// (mirrors composition.simulation_mission_groups one-for-one) and consumes
// `runs` in order via a shared cursor passed through buildMissionNode().
YAML::Node buildSimulationsNode(const std::vector<types::SimulationResult>& runs,
                                const CompositionFilePaths& file_paths) {

    YAML::Node simulations_node;
    std::size_t run_index = 0;

    for (const auto& [sim_ref, mission_refs] : file_paths.simulation_mission_paths) {

        YAML::Node sim_node;
        sim_node["simulation_config"] = sim_ref.path;

        YAML::Node missions_node;
        for (const ReferencedConfigFile& mission_ref : mission_refs) {
            missions_node.push_back(
                buildMissionNode(
                    mission_ref.path,
                    runs,
                    run_index,
                    file_paths.drone_paths,
                    file_paths.lidar_paths));
        }

        sim_node["missions"] = missions_node;
        simulations_node.push_back(sim_node);
    }

    return simulations_node;
}

// total_runs/scored_runs/error_runs counts, plus average/min/max computed
// over scored runs only (score >= 0), excluding error runs (score == -1).
YAML::Node buildSummaryNode(const std::vector<types::SimulationResult>& runs) {
    std::size_t error_runs = 0;
    std::vector<double> scored;
    scored.reserve(runs.size());
    for (const types::SimulationResult& result : runs) {
        if (result.mission_score < 0.0) {
            ++error_runs;
        } else {
            scored.push_back(result.mission_score);
        }
    }

    YAML::Node node;
    node["total_runs"] = runs.size();
    node["scored_runs"] = scored.size();
    node["error_runs"] = error_runs;

    constexpr double kErrorScoreSentinel = -1.0;
    if (scored.empty()) {
        node["average_score"] = kErrorScoreSentinel;
        node["min_score"] = kErrorScoreSentinel;
        node["max_score"] = kErrorScoreSentinel;
    } else {
        const double sum = std::accumulate(scored.begin(), scored.end(), 0.0);
        node["average_score"] = sum / static_cast<double>(scored.size());
        node["min_score"] = *std::min_element(scored.begin(), scored.end());
        node["max_score"] = *std::max_element(scored.begin(), scored.end());
    }
    return node;
}

} // namespace

void writeSimulationOutput(const types::SimulationManagerReport& report, const CompositionFilePaths& file_paths,
                          const std::filesystem::path& output_yaml_path) {
    YAML::Node score_range_node;
    score_range_node["min"] = std::get<0>(report.score_range);
    score_range_node["max"] = std::get<1>(report.score_range);

    YAML::Node score_report;
    score_report["composition_file"] = report.composition_file.string();
    score_report["generated_at_utc"] = report.generated_at_utc;
    score_report["metric"] = report.metric;
    score_report["score_range"] = score_range_node;
    score_report["error_score"] = report.error_score;
    score_report["summary"] = buildSummaryNode(report.runs);
    score_report["simulations"] = buildSimulationsNode(report.runs, file_paths);

    YAML::Node root;
    root["score_report"] = score_report;

    std::ofstream out(output_yaml_path);
    out << root;
}

ComponentRunTotals computeComponentTotals(const std::string& component_name,
                                        const types::SimulationManagerReport& report) {

    double total_score = 0.0;
    std::size_t total_steps = 0;
    
    // Following a forum clarification, only non-negative scores are included in total_score.
    for (const types::SimulationResult& result : report.runs) {
    if (result.mission_score >= 0.0) {
        total_score += result.mission_score;
    }
    total_steps += result.mission_results.at(0).steps;
    }

    return ComponentRunTotals{
        component_name,
        total_score,
        total_steps};
}

namespace {

std::vector<std::vector<const ComponentRunTotals*>> groupBySameResult(
    const std::vector<ComponentRunTotals>& totals) {
    std::vector<std::vector<const ComponentRunTotals*>> groups;
    for (const ComponentRunTotals& item : totals) {
        auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& group) {
            return group.front()->total_score == item.total_score &&
                   group.front()->total_steps == item.total_steps;
        });
        if (it != groups.end()) {
            it->push_back(&item);
        } else {
            groups.push_back({&item});
        }
    }
    // Sorted by group size descending. Secondary key for size ties is unspecified by the doc --
    // using total_score descending as a stable, deterministic tiebreaker.
    std::stable_sort(groups.begin(), groups.end(), [](const auto& a, const auto& b) {
        if (a.size() != b.size()) return a.size() > b.size();
        return a.front()->total_score > b.front()->total_score;
    });
    return groups;
}

} // namespace

void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<ComponentRunTotals>& mission_control_totals,
                            const std::vector<std::string>& failed_mission_controls,
                            const std::filesystem::path& output_yaml_path) {
    YAML::Node comparative;
    comparative["composition_file"] = composition_file.string();
    comparative["mission_control_folder"] = mission_control_folder.string();
    comparative["generated_at_utc"] = currentUtcTimestamp();  // moved/duplicated helper, see note

    YAML::Node results_summary;
    for (const auto& group : groupBySameResult(mission_control_totals)) {
        YAML::Node group_node;
        YAML::Node names;
        for (const ComponentRunTotals* item : group) names.push_back(item->component_name);
        group_node["same_results"] = names;
        group_node["total_score"] = group.front()->total_score;
        group_node["total_steps"] = group.front()->total_steps;
        results_summary.push_back(group_node);
    }
    comparative["results_summary"] = results_summary;

    YAML::Node errors_node;
    for (const std::string& name : failed_mission_controls) errors_node.push_back(name);
    comparative["errors"] = errors_node;

    YAML::Node root;
    root["comparative_report"] = comparative;
    std::ofstream out(output_yaml_path);
    out << root;
}

void writeCompetitiveReport(const std::filesystem::path& composition_file,
                           const std::filesystem::path& mission_control,
                           const std::vector<ComponentRunTotals>& algorithm_totals,
                           const std::vector<std::string>& failed_algorithms,
                           const std::filesystem::path& output_yaml_path) {
    std::vector<ComponentRunTotals> sorted = algorithm_totals;
    std::stable_sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        if (a.total_score != b.total_score) return a.total_score > b.total_score;
        return a.total_steps < b.total_steps;
    });

    YAML::Node competitive;
    competitive["composition_file"] = composition_file.string();
    competitive["mission_control"] = mission_control.string();
    competitive["generated_at_utc"] = currentUtcTimestamp();

    YAML::Node results_summary;
    for (const ComponentRunTotals& item : sorted) {
        YAML::Node entry;
        entry["algorithm"] = item.component_name;
        entry["total_score"] = item.total_score;
        entry["total_steps"] = item.total_steps;
        results_summary.push_back(entry);
    }
    competitive["results_summary"] = results_summary;

    YAML::Node errors_node;
    for (const std::string& name : failed_algorithms) errors_node.push_back(name);
    competitive["errors"] = errors_node;

    YAML::Node root;
    root["competitive_report"] = competitive;
    std::ofstream out(output_yaml_path);
    out << root;
}

} // namespace simulator
