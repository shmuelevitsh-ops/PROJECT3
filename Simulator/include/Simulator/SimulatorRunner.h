#pragma once

#include <Simulator/CliOptions.h>

#include <cstddef>
#include <filesystem>

namespace simulator {

// Runs the simulation in comparative mode and writes the comparative report.
// Returns the number of components that completed successfully.
[[nodiscard]] std::size_t runComparative(const ComparativeOptions& options,
                                         const std::filesystem::path& results_dir);

// Runs the simulation in competition mode and writes the competition report.
// Returns the number of components that completed successfully.
[[nodiscard]] std::size_t runCompetition(const CompetitionOptions& options,
                                         const std::filesystem::path& results_dir);

// Determines how many worker threads to use.
// Returns 0 when the work should run sequentially on the calling thread.
// Exposed here so it can be tested directly.
[[nodiscard]] std::size_t computeWorkerCount(const std::optional<int>& num_threads,
                                             std::size_t work_items);

} // namespace simulator
