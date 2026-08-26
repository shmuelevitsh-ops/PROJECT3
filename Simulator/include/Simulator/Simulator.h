#pragma once

#include <Simulator/CliOptions.h>

#include <cstddef>
#include <filesystem>
#include <variant>

namespace simulator {

// A single configured simulator run: the mode-specific options selected on the command line
// (comparative or competition) plus the results directory to write into.
// Orchestrates that run: loads the components, runs each one over the shared composition,
// collects the results and writes the final report. Running the components sequentially or
// concurrently is delegated to ParallelExecutor.
class Simulator {
public:
    Simulator(std::variant<ComparativeOptions, CompetitionOptions> options, std::filesystem::path results_dir);

    // Runs the configured simulation (comparative or competition, per the stored options) and
    // writes the corresponding report. Returns the number of components that completed successfully.
    [[nodiscard]] std::size_t run() const;

private:
    // Runs the simulation in comparative mode and writes the comparative report.
    // Reads its ComparativeOptions from options_.
    [[nodiscard]] std::size_t runComparative() const;

    // Runs the simulation in competition mode and writes the competition report.
    // Reads its CompetitionOptions from options_.
    [[nodiscard]] std::size_t runCompetition() const;

    std::variant<ComparativeOptions, CompetitionOptions> options_;
    std::filesystem::path results_dir_;
};

} // namespace simulator
