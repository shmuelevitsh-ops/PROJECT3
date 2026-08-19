#include <Simulator/ConfigLoader.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace simulator {

namespace common_types = common::types;
namespace isq = common::isq;

using common::Position3D;
using common::cm;
using common::deg;
using common::horizontal_angle;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// Parses x/y/height min-max boundaries from the mission YAML (to MappingBounds)
common_types::MappingBounds parseBoundaries(const YAML::Node& boundaries_node) {
    const auto axis = [](const YAML::Node& axis_node) {
        return std::make_pair(axis_node["min_cm"].as<double>(), axis_node["max_cm"].as<double>());
    };
    const auto [min_x, max_x] = axis(boundaries_node["x_boundary"]);
    const auto [min_y, max_y] = axis(boundaries_node["y_boundary"]);
    const auto [min_h, max_h] = axis(boundaries_node["height_boundary"]);

    return common_types::MappingBounds{
        min_x * x_extent[cm], max_x * x_extent[cm],
        min_y * y_extent[cm], max_y * y_extent[cm],
        min_h * z_extent[cm], max_h * z_extent[cm]};
}

// Parses simulation_config.yaml's `map_axes_offset`.
Position3D parseOffset(const YAML::Node& offset_node) {
    return Position3D{
        offset_node["x_offset"].as<double>() * x_extent[cm],
        offset_node["y_offset"].as<double>() * y_extent[cm],
        offset_node["height_offset"].as<double>() * z_extent[cm]};
}

// Parses simulation_config.yaml's `initial_drone_position`.
Position3D parsePosition3D(const YAML::Node& position_node) {
    return Position3D{
        position_node["x_cm"].as<double>() * x_extent[cm],
        position_node["y_cm"].as<double>() * y_extent[cm],
        position_node["height_cm"].as<double>() * z_extent[cm]};
}

// Resolves referenced config paths: absolute first,
// then relative to the composition, then relative to the CWD.
std::filesystem::path resolveReferencedFile(const std::filesystem::path& base_dir,
                                            const std::filesystem::path& raw_path) {
    if (raw_path.is_absolute()) {
        return raw_path;
    }
    const std::filesystem::path relative_to_composition = base_dir / raw_path;
    if (std::filesystem::exists(relative_to_composition)) {
        return relative_to_composition;
    }
    return raw_path; // fall back to CWD-relative resolution
}

// Used for drone_configs and lidar_configs.
// If one referenced file cannot be loaded, that entry is logged and skipped.
// Parses a referenced config and skips it if loading fails.
// Templated so the same logic works for both drone and lidar parsers.
template <typename ParseFn>
auto parseOrSkip(const std::filesystem::path& base_dir, const std::string& raw_path, ParseFn parse)
    -> std::optional<std::invoke_result_t<ParseFn, const std::filesystem::path&>> {
    // invoke_result_t gets the return type of ParseFn when called with a path.
    const std::filesystem::path resolved = resolveReferencedFile(base_dir, raw_path);
    try {
        return parse(resolved);
    } catch (const std::exception& e) {
        std::cerr << "parseCompositionData: failed to load referenced file '" << raw_path
                  << "' (resolved: '" << resolved.string() << "'): " << e.what()
                  << " — skipping this entry.\n";
        return std::nullopt;
    }
}

using MissionParseResult = std::pair<
    common_types::MissionConfigData,
    std::optional<common_types::ErrorRef>>;

// Parses a mission config.
// On failure, returns a placeholder mission together with the load error.
MissionParseResult parseMissionOrPlaceholder(const std::filesystem::path& base_dir, const std::string& raw_path) {
    const std::filesystem::path resolved = resolveReferencedFile(base_dir, raw_path);
    try {
        return {parseMissionConfig(resolved), std::nullopt};
    } catch (const std::exception& e) {
        std::cerr << "parseCompositionData: failed to load referenced file '" << raw_path
                  << "' (resolved: '" << resolved.string() << "'): " << e.what()
                  << " — marking this mission as a load failure (score -1).\n";
         return {common_types::MissionConfigData{},
                std::optional<common_types::ErrorRef>{
                    common_types::ErrorRef{
                        "MISSION_CONFIG_LOAD_FAILED",
                        e.what()}}};
    }
}

} // namespace

// Parses drone properties from a drone_config YAML file.
common_types::DroneConfigData parseDroneConfig(const std::filesystem::path& path) {
    const YAML::Node node = YAML::LoadFile(path.string())["drone_config"];

    common_types::DroneConfigData config;
    const double dimensions_cm = node["dimensions_cm"].as<double>();
    config.radius = (dimensions_cm / 2.0) * isq::length[cm]; // dimensions_cm is the sphere diameter
    config.max_rotate = node["max_rotate_deg"].as<double>() * horizontal_angle[deg];
    config.max_advance = node["max_advance_cm"].as<double>() * isq::length[cm];
    config.max_elevate = node["max_elevate_cm"].as<double>() * isq::length[cm];
    return config;
}

common_types::MissionConfigData parseMissionConfig(const std::filesystem::path& path) {
    const YAML::Node node = YAML::LoadFile(path.string())["mission_config"];

    common_types::MissionConfigData config;
    config.max_steps = node["max_steps"].as<std::size_t>();
    config.mission_bounds = parseBoundaries(node["boundaries"]);
    config.gps_resolution = node["gps_resolution_cm"].as<double>() * isq::length[cm];

    // Optional resolution request.
    // The value is preserved here; validation and handling happen downstream.
    if (const YAML::Node factor_node = node["output_mapping_resolution_factor"]) {
        config.output_mapping_resolution_factor = factor_node.as<double>();
        if (config.output_mapping_resolution_factor < 1.0) {
            std::cerr << "parseMissionConfig: output_mapping_resolution_factor < 1 in '" << path.string()
                      << "' — value preserved as parsed; will be ignored downstream.\n";
        }
    } else {
        config.output_mapping_resolution_factor = 1.0;
        std::cerr << "parseMissionConfig: output_mapping_resolution_factor missing in '" << path.string()
                  << "' — defaulting to 1.\n";
    }
    return config;
}

// Parses LiDAR properties from a lidar_config YAML file.
common_types::LidarConfigData parseLidarConfig(const std::filesystem::path& path) {
    const YAML::Node node = YAML::LoadFile(path.string())["lidar_config"];

    common_types::LidarConfigData config;
    config.z_min = node["z_min_cm"].as<double>() * isq::length[cm];
    config.z_max = node["z_max_cm"].as<double>() * isq::length[cm];
    config.d = node["d_cm"].as<double>() * isq::length[cm];
    config.fov_circles = node["fov_circles"].as<std::size_t>();
    return config;
}

// Parses simulation setup and resolves its map path.
types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path) {
    const YAML::Node node = YAML::LoadFile(path.string())["simulation_config"];

    types::SimulationConfigData config;
    const std::filesystem::path raw_map = node["map_filename"].as<std::string>();
    config.map_filename = raw_map.is_absolute() ? raw_map : path.parent_path() / raw_map;
    config.map_resolution = node["map_resolution_cm"].as<double>() * isq::length[cm];
    config.map_offset = parseOffset(node["map_axes_offset"]);
    config.initial_drone_position = parsePosition3D(node["initial_drone_position"]);
    config.initial_angle = node["initial_angle_deg"].as<double>() * horizontal_angle[deg];
    return config;
}

namespace {
// Parses a drone/lidar config list while keeping values and paths aligned.
// Invalid entries are skipped from both vectors.
template <typename ParseFn>
void parseConfigList(const YAML::Node& list_node, const std::filesystem::path& base_dir, ParseFn parse,
                     std::vector<std::invoke_result_t<ParseFn, const std::filesystem::path&>>& out_values,
                     std::vector<std::string>& out_paths) {
    for (const YAML::Node& entry : list_node) {
        const std::string raw_path = entry.as<std::string>();
        if (auto value = parseOrSkip(base_dir, raw_path, parse)) {
            out_values.push_back(*value);
            out_paths.push_back(raw_path);
        }
    }
}

// Parses one simulation and its missions while preserving load failures.
void parseSimulationMissionGroup(const YAML::Node& sim_entry, const std::filesystem::path& base_dir,
                                 types::SimulationCompositionData& composition,
                                 CompositionFilePaths& file_paths) {
    const std::string sim_raw_path = sim_entry["simulation_config"].as<std::string>();
    const std::filesystem::path sim_resolved = resolveReferencedFile(base_dir, sim_raw_path);

    types::SimulationConfigData simulation;

    std::optional<common_types::ErrorRef> simulation_load_error;

    try {
        simulation = parseSimulationConfig(sim_resolved);
    } catch (const std::exception& e) {
        std::cerr << "parseCompositionData: failed to load referenced file '" << sim_raw_path
                  << "' (resolved: '" << sim_resolved.string() << "'): " << e.what()
                  << " — marking every scenario in this group as a load failure (score -1).\n";
        simulation = types::SimulationConfigData{};
        simulation_load_error = common_types::ErrorRef{"SIMULATION_CONFIG_LOAD_FAILED", e.what()};
    }

    std::vector<common_types::MissionConfigData> missions;
    std::vector<ReferencedConfigFile> mission_refs;
    for (const YAML::Node& mission_entry : sim_entry["mission_configs"]) {
        const std::string mission_raw_path = mission_entry.as<std::string>();
        if (simulation_load_error) {
            // The simulation_config itself failed, so none of its missions
            // can run. 
            missions.push_back(common_types::MissionConfigData{});
            mission_refs.push_back(ReferencedConfigFile{mission_raw_path, std::nullopt});
            continue;
        }

        auto [mission, mission_load_error] = parseMissionOrPlaceholder(base_dir, mission_raw_path);

        missions.push_back(std::move(mission));

        mission_refs.push_back(ReferencedConfigFile{mission_raw_path, std::move(mission_load_error)});
    }

    composition.simulation_mission_groups.emplace_back(std::move(simulation), std::move(missions));
    file_paths.simulation_mission_paths.emplace_back(ReferencedConfigFile
                                                    {sim_raw_path, std::move(simulation_load_error)},
                                                     std::move(mission_refs));
    }

} // namespace

// Entry function
// Parses the composition YAML and all referenced config files.
// Skips invalid drone/lidar configs and preserves simulation/mission failures.
ParsedComposition parseCompositionData(const std::filesystem::path& path) {
    const YAML::Node node = YAML::LoadFile(path.string())["simulation_compositions"];
    const std::filesystem::path base_dir = path.parent_path();

    types::SimulationCompositionData composition;
    composition.composition_file = path;
    CompositionFilePaths file_paths;

    parseConfigList(node["drone_configs"], base_dir, parseDroneConfig, composition.drone_configs, file_paths.drone_paths);
    parseConfigList(node["lidar_configs"], base_dir, parseLidarConfig, composition.lidar_configs, file_paths.lidar_paths);

    for (const YAML::Node& sim_entry : node["simulations"]) {
        parseSimulationMissionGroup(sim_entry, base_dir, composition, file_paths);
    }

    return ParsedComposition{std::move(composition), std::move(file_paths)};
}

} // namespace simulator
