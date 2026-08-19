#pragma once

#include <filesystem>
#include <optional>
#include <variant>
#include <vector>

namespace simulator {

struct ComparativeOptions {
    std::filesystem::path simulation_composition_file;
    std::filesystem::path mission_control_folder;
    std::filesystem::path algorithm_so_file;
    std::vector<std::filesystem::path> mission_control_libraries; // *.so directly under mission_control_folder, non-empty
    std::optional<int> num_threads;
    bool verbose = false;
};

struct CompetitionOptions {
    std::filesystem::path simulation_composition_file;
    std::filesystem::path mission_control_so_file;
    std::filesystem::path algorithms_folder;
    std::vector<std::filesystem::path> algorithm_libraries; // *.so directly under algorithms_folder, non-empty
    std::optional<int> num_threads;
    bool verbose = false;
};

// Parses and validates the simulator command-line arguments.
// Returns the selected mode options, or nullopt after reporting validation errors.
[[nodiscard]] std::optional<std::variant<ComparativeOptions, CompetitionOptions>>
parseCliOptions(int argc, char** argv);

} // namespace simulator
