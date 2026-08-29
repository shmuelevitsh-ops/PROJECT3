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

// Inserts a blank line between two sibling block-map keys. yaml-cpp's default
// newline-before-key logic is suppressed once a YAML::Newline has already been
// emitted in the enclosing map (it counts as "node begun"), so a single Newline
// there only ever produces a normal line break -- two are required to actually
// open up a blank line. Block-sequence items don't have this quirk: a single
// Newline before an item already yields a blank line (see emitRunsSeq below).
void blankLineBetweenMapKeys(YAML::Emitter& out) {
    out << YAML::Newline << YAML::Newline;
}

// One run: { drone_config, lidar_config, status, steps, score, error_ref? }.
// error_ref is printed whenever errors is non-empty, regardless of status —
// preserves diagnostics even for a "completed" run with a recorded warning
// such as unmappable voxels, not just for status == "error".
void emitRun(YAML::Emitter& out, const types::SimulationResult& result, const std::string& drone_path,
            const std::string& lidar_path) {
    const common_types::MissionRunResult& mission_result = result.mission_results.at(0);

    out << YAML::BeginMap;
    out << YAML::Key << "drone_config" << YAML::Value << YAML::DoubleQuoted << drone_path;
    out << YAML::Key << "lidar_config" << YAML::Value << YAML::DoubleQuoted << lidar_path;
    out << YAML::Key << "status" << YAML::Value << YAML::DoubleQuoted << statusToString(mission_result.status);
    out << YAML::Key << "steps" << YAML::Value << mission_result.steps;
    out << YAML::Key << "score" << YAML::Value << result.mission_score;

    if (!mission_result.errors.empty()) {
        out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "code" << YAML::Value << YAML::DoubleQuoted << mission_result.errors.front().code;
        out << YAML::EndMap;
    }
    out << YAML::EndMap;
}

// The `runs:` sequence for one mission -- a blank line follows every run entry,
// including the last one (it doubles as the separator before whatever comes next:
// the following mission, or the following simulation), matching the reference format.
void emitRunsSeq(YAML::Emitter& out, const std::vector<types::SimulationResult>& runs, std::size_t& run_index,
                 const std::vector<std::string>& drone_paths, const std::vector<std::string>& lidar_paths) {
    out << YAML::BeginSeq;
    for (const std::string& drone_path : drone_paths) {
        for (const std::string& lidar_path : lidar_paths) {
            emitRun(out, runs.at(run_index++), drone_path, lidar_path);
            out << YAML::Newline;
        }
    }
    out << YAML::EndSeq;
}

// One mission: { mission_config, resolution_cm, resolution_request_status, runs }.
// Consumes exactly drone_paths.size() * lidar_paths.size() entries from
// `runs`, starting at `run_index` (advanced by reference), matching
// SimulationManager::run()'s nested-loop order. resolution_cm and
// resolution_request_status are the same across every run in one mission
// (both decided once per mission, independent of drone/lidar), so they're
// read off the first run about to be consumed.
void emitMission(YAML::Emitter& out, const std::string& mission_path,
                 const std::vector<types::SimulationResult>& runs, std::size_t& run_index,
                 const std::vector<std::string>& drone_paths, const std::vector<std::string>& lidar_paths) {
    const types::SimulationResult& first_result = runs.at(run_index);

    out << YAML::BeginMap;
    out << YAML::Key << "mission_config" << YAML::Value << YAML::DoubleQuoted << mission_path;
    out << YAML::Key << "resolution_cm" << YAML::Value
        << first_result.output_map_config.resolution.force_numerical_value_in(cm);
    out << YAML::Key << "resolution_request_status" << YAML::Value
        << resolutionStatusToString(first_result.resolution_request_status);
    out << YAML::Key << "runs" << YAML::Value;
    emitRunsSeq(out, runs, run_index, drone_paths, lidar_paths);
    out << YAML::EndMap;
}

// The `simulations:` hierarchy: walks file_paths.simulation_mission_paths
// (mirrors composition.simulation_mission_groups one-for-one) and consumes
// `runs` in order via a shared cursor passed through emitMission().
void emitSimulationsSeq(YAML::Emitter& out, const std::vector<types::SimulationResult>& runs,
                        const CompositionFilePaths& file_paths) {
    out << YAML::BeginSeq;
    std::size_t run_index = 0;

    for (const auto& [sim_ref, mission_refs] : file_paths.simulation_mission_paths) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_config" << YAML::Value << YAML::DoubleQuoted << sim_ref.path;

        out << YAML::Key << "missions" << YAML::Value << YAML::BeginSeq;
        for (const ReferencedConfigFile& mission_ref : mission_refs) {
            emitMission(out, mission_ref.path, runs, run_index, file_paths.drone_paths, file_paths.lidar_paths);
        }
        out << YAML::EndSeq;

        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
}

// total_runs/scored_runs/error_runs counts, plus average/min/max computed
// over scored runs only (score >= 0), excluding error runs (score == -1).
void emitSummary(YAML::Emitter& out, const std::vector<types::SimulationResult>& runs) {
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

    out << YAML::BeginMap;
    out << YAML::Key << "total_runs" << YAML::Value << runs.size();
    out << YAML::Key << "scored_runs" << YAML::Value << scored.size();
    out << YAML::Key << "error_runs" << YAML::Value << error_runs;

    constexpr double kErrorScoreSentinel = -1.0;
    if (scored.empty()) {
        out << YAML::Key << "average_score" << YAML::Value << kErrorScoreSentinel;
        out << YAML::Key << "min_score" << YAML::Value << kErrorScoreSentinel;
        out << YAML::Key << "max_score" << YAML::Value << kErrorScoreSentinel;
    } else {
        const double sum = std::accumulate(scored.begin(), scored.end(), 0.0);
        out << YAML::Key << "average_score" << YAML::Value << sum / static_cast<double>(scored.size());
        out << YAML::Key << "min_score" << YAML::Value << *std::min_element(scored.begin(), scored.end());
        out << YAML::Key << "max_score" << YAML::Value << *std::max_element(scored.begin(), scored.end());
    }
    out << YAML::EndMap;
}

} // namespace

void writeSimulationOutput(const types::SimulationManagerReport& report, const CompositionFilePaths& file_paths,
                          const std::filesystem::path& output_yaml_path) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << YAML::DoubleQuoted << report.composition_file.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << YAML::DoubleQuoted << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << YAML::DoubleQuoted << report.metric;

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "score_range" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range);
    out << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range);
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;
    out << YAML::EndMap;

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "summary" << YAML::Value;
    emitSummary(out, report.runs);

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "simulations" << YAML::Value;
    emitSimulationsSeq(out, report.runs, file_paths);

    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // root

    std::ofstream file(output_yaml_path);
    file << out.c_str() << "\n";
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

// Flow-style ["a", "b"] list of double-quoted strings; [] when empty.
void emitFlowStringList(YAML::Emitter& out, const std::vector<std::string>& values) {
    out << YAML::Flow << YAML::BeginSeq;
    for (const std::string& value : values) {
        out << YAML::DoubleQuoted << value;
    }
    out << YAML::EndSeq;
}

} // namespace

void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<ComponentRunTotals>& mission_control_totals,
                            const std::vector<std::string>& failed_mission_controls,
                            const std::filesystem::path& output_yaml_path) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "comparative_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << YAML::DoubleQuoted << composition_file.string();
    out << YAML::Key << "mission_control_folder" << YAML::Value << YAML::DoubleQuoted
        << mission_control_folder.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << YAML::DoubleQuoted << currentUtcTimestamp();

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "results_summary" << YAML::Value << YAML::BeginSeq;
    for (const auto& group : groupBySameResult(mission_control_totals)) {
        out << YAML::BeginMap;
        out << YAML::Key << "same_results" << YAML::Value;
        std::vector<std::string> names;
        names.reserve(group.size());
        for (const ComponentRunTotals* item : group) names.push_back(item->component_name);
        emitFlowStringList(out, names);
        out << YAML::Key << "total_score" << YAML::Value << group.front()->total_score;
        out << YAML::Key << "total_steps" << YAML::Value << group.front()->total_steps;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "errors" << YAML::Value;
    emitFlowStringList(out, failed_mission_controls);

    out << YAML::EndMap; // comparative_report
    out << YAML::EndMap; // root

    std::ofstream file(output_yaml_path);
    file << out.c_str() << "\n";
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

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "competitive_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << YAML::DoubleQuoted << composition_file.string();
    out << YAML::Key << "mission_control" << YAML::Value << YAML::DoubleQuoted << mission_control.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << YAML::DoubleQuoted << currentUtcTimestamp();

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "results_summary" << YAML::Value << YAML::BeginSeq;
    for (const ComponentRunTotals& item : sorted) {
        out << YAML::BeginMap;
        out << YAML::Key << "algorithm" << YAML::Value << YAML::DoubleQuoted << item.component_name;
        out << YAML::Key << "total_score" << YAML::Value << item.total_score;
        out << YAML::Key << "total_steps" << YAML::Value << item.total_steps;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    blankLineBetweenMapKeys(out);
    out << YAML::Key << "errors" << YAML::Value;
    emitFlowStringList(out, failed_algorithms);

    out << YAML::EndMap; // competitive_report
    out << YAML::EndMap; // root

    std::ofstream file(output_yaml_path);
    file << out.c_str() << "\n";
}

} // namespace simulator
