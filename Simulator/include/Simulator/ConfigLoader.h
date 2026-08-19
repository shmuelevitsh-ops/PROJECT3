#pragma once

#include <Common/Types.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <tuple>
#include <vector>
#include <optional>

namespace simulator {

/// Parses YAML input files into their corresponding configuration structs.
// Errors are handled by the caller.
[[nodiscard]] common::types::DroneConfigData parseDroneConfig(
    const std::filesystem::path& path);

[[nodiscard]] common::types::MissionConfigData parseMissionConfig(
    const std::filesystem::path& path);

[[nodiscard]] common::types::LidarConfigData parseLidarConfig(
    const std::filesystem::path& path);

[[nodiscard]] types::SimulationConfigData parseSimulationConfig(
    const std::filesystem::path& path);

// Stores the source path of a simulation/mission config
// and the error if that file failed to load.
struct ReferencedConfigFile {
    std::string path;
    std::optional<common::types::ErrorRef> load_error{};
};

// Keeps input file paths aligned with their parsed configs in the composition.
struct CompositionFilePaths {
    std::vector<
        std::tuple<
            ReferencedConfigFile,
            std::vector<ReferencedConfigFile>>>
        simulation_mission_paths;

    // Invalid drone/lidar configs are skipped rather than represented by
    // placeholders, so their entries only need to store the original paths.
    std::vector<std::string> drone_paths;
    std::vector<std::string> lidar_paths;
};

// Parsed configuration data together with its original input-file metadata.
struct ParsedComposition {
    types::SimulationCompositionData composition;
    CompositionFilePaths file_paths;
};

// Parses the composition YAML and its referenced configs.
[[nodiscard]] ParsedComposition parseCompositionData(
    const std::filesystem::path& path);

} // namespace simulator
