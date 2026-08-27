#pragma once

#include <Simulator/CliOptions.h>

#include <cstddef>
#include <filesystem>
#include <variant>

namespace simulator {

// Orchestrates a simulator run in comparative or competition mode.
class Simulator {
public:
    Simulator(std::variant<ComparativeOptions, CompetitionOptions> options, std::filesystem::path results_dir);

    // Runs the selected mode and returns the number of successful components.
    [[nodiscard]] std::size_t run() const;

private:
    // Runs comparative mode: fixed Algorithm, varying MissionControls
    [[nodiscard]] std::size_t runComparative() const;

    // Runs competition mode: fixed MissionControl, varying Algorithms.
    [[nodiscard]] std::size_t runCompetition() const;

    std::variant<ComparativeOptions, CompetitionOptions> options_;
    std::filesystem::path results_dir_;
};

} // namespace simulator
