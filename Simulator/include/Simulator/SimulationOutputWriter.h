#pragma once

#include <Simulator/ConfigLoader.h>       // CompositionFilePaths, ReferencedConfigFile
#include <Simulator/SimulationTypes.h>    // simulator::types::SimulationManagerReport

#include <filesystem>
#include <string>
#include <vector>
#include <cstddef>

namespace simulator {

// One component's (mission-control .so, or algorithm .so) aggregate over a full composition
// sweep -- sum of mission_score / mission_results.front().steps across every run in that
// component's own SimulationManagerReport. There is deliberately no "successfully_loaded" flag
// here: a component that could never be dlopen'd/run at all has no SimulationManagerReport to
// aggregate, so the caller tracks that case as a bare name in the separate `errors` list taken
// by writeComparativeReport/writeCompetitiveReport below, never as a ComponentRunTotals.
struct ComponentRunTotals {
    std::string component_name;   // e.g. "manager1.so" / "algorithm3.so" -- basename, not full path
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

// Sums mission_score/steps across every run in `report`. Assumption: "total_score"/"total_steps"
// in the comparative/competitive YAML are additive sums over the component's full sweep, not an
// average -- the assignment doc names them distinctly from the per-component file's existing
// average_score, but doesn't define the reduction; confirm before relying on this.
[[nodiscard]] ComponentRunTotals computeComponentTotals(const std::string& component_name,
                                                        const types::SimulationManagerReport& report);

// Per-component detailed file, unchanged in shape from Assignment 2. composition_file is now
// read from report.composition_file (published field, not a separate parameter -- see .cpp notes).
void writeSimulationOutput(const types::SimulationManagerReport& report,
                          const CompositionFilePaths& file_paths,
                          const std::filesystem::path& output_yaml_path);

// Comparative-mode top-level report: groups mission_control_totals by identical
// (total_score, total_steps) into same_results, sorted by group size descending.
void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<ComponentRunTotals>& mission_control_totals,
                            const std::vector<std::string>& failed_mission_controls,
                            const std::filesystem::path& output_yaml_path);

// Competitive-mode top-level report: ranks algorithm_totals by total_score descending, then
// total_steps ascending. No grouping -- every algorithm gets its own results_summary entry.
void writeCompetitiveReport(const std::filesystem::path& composition_file,
                           const std::filesystem::path& mission_control,
                           const std::vector<ComponentRunTotals>& algorithm_totals,
                           const std::vector<std::string>& failed_algorithms,
                           const std::filesystem::path& output_yaml_path);

} // namespace simulator