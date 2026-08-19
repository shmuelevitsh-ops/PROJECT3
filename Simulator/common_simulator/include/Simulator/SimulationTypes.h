#pragma once

#include <Common/Types.h>

#include <filesystem>
#include <string>
#include <tuple>
#include <vector>

namespace simulator::types {

struct SimulationConfigData {
    std::filesystem::path map_filename{};
    common::PhysicalLength map_resolution{};
    common::Position3D map_offset{};
    common::Position3D initial_drone_position{};
    common::HorizontalAngle initial_angle{};
};

struct SimulationCompositionData {
    std::filesystem::path composition_file{};
    std::vector<std::tuple<SimulationConfigData, std::vector<common::types::MissionConfigData>>> simulation_mission_groups{};
    std::vector<common::types::DroneConfigData> drone_configs{};
    std::vector<common::types::LidarConfigData> lidar_configs{};
};

enum class ResolutionRequestStatus { Accepted, Ignored, IgnoredTooSmall };

struct SimulationResult {
    SimulationConfigData simulation_config{};
    common::types::MissionConfigData mission_config{};
    ResolutionRequestStatus resolution_request_status = ResolutionRequestStatus::Ignored;
    std::vector<common::types::MissionRunResult> mission_results{};
    std::filesystem::path output_map_file{};
    common::types::MapConfig output_map_config{};
    double mission_score = 0.0;
};

struct SimulationManagerReport {
    std::filesystem::path composition_file{};
    std::string generated_at_utc{};
    std::string metric{};
    std::tuple<double, double> score_range{};
    int error_score = -1;
    std::vector<SimulationResult> runs{};
};

} // namespace simulator::types
