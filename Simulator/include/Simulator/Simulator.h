#pragma once

#include <Simulator/CliOptions.h>

#include <cstddef>
#include <filesystem>

namespace simulator {

// Orchestrates a whole simulator run: loads the components, runs each one over the
// shared composition, collects the results and writes the final report.
// Running the components sequentially or concurrently is delegated to ParallelExecutor.
class Simulator {
public:
    // Runs the simulation in comparative mode and writes the comparative report.
    // Returns the number of components that completed successfully.
    [[nodiscard]] std::size_t runComparative(const ComparativeOptions& options,
                                             const std::filesystem::path& results_dir) const;

    // Runs the simulation in competition mode and writes the competition report.
    // Returns the number of components that completed successfully.
    [[nodiscard]] std::size_t runCompetition(const CompetitionOptions& options,
                                             const std::filesystem::path& results_dir) const;
};

} // namespace simulator
