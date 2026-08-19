#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>

#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace simulator {

namespace common_types = common::types;

using common::PhysicalLength;

namespace {

// Derives the resolution_request_status by comparing the requested output
// resolution with the resolution actually used for the output map.
types::ResolutionRequestStatus resolutionRequestStatus(
    const types::SimulationConfigData& simulation_config,
    const common_types::MissionConfigData& mission_config) {

    if (mission_config.output_mapping_resolution_factor < 1.0) {
        return types::ResolutionRequestStatus::IgnoredTooSmall;
    }

    const PhysicalLength requested_resolution =
        mission_config.gps_resolution * mission_config.output_mapping_resolution_factor;

    if (requested_resolution == simulation_config.map_resolution) {
        return types::ResolutionRequestStatus::Accepted;
    }

    return types::ResolutionRequestStatus::Ignored;
}

} // namespace

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const common::IMap3D> hidden_map,
                                    std::unique_ptr<common::IMutableMap3D> output_map,
                                    std::unique_ptr<common::IGPS> gps,
                                    std::unique_ptr<common::IDroneMovement> movement,
                                    std::unique_ptr<common::ILidar> lidar,
                                    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                                    std::unique_ptr<common::IMissionControl> mission_control,
                                    types::SimulationConfigData simulation_config,
                                    common_types::MissionConfigData mission_config,
                                    std::filesystem::path output_map_file)
                                    : hidden_map_(std::move(hidden_map)),
                                    output_map_(std::move(output_map)),
                                    gps_(std::move(gps)),
                                    movement_(std::move(movement)),
                                    lidar_(std::move(lidar)),
                                    mapping_algorithm_(std::move(mapping_algorithm)),
                                    mission_control_(std::move(mission_control)),
                                    simulation_config_(std::move(simulation_config)),
                                    mission_config_(std::move(mission_config)),
                                    output_map_file_(std::move(output_map_file)) {

    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ || !lidar_ ||
        !mapping_algorithm_ || !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    const common_types::MappingBounds& mb = mission_config_.mission_bounds;
    const common_types::MappingBounds& hb = hidden_map_->getMapConfig().boundaries;

    const bool mission_valid =
        mb.min_x < mb.max_x && mb.min_y < mb.max_y && mb.min_height < mb.max_height;

    const bool hidden_valid =
        hb.min_x < hb.max_x && hb.min_y < hb.max_y && hb.min_height < hb.max_height;

    if (mission_valid && hidden_valid &&
        (mb.min_x >= hb.max_x || mb.max_x <= hb.min_x ||
         mb.min_y >= hb.max_y || mb.max_y <= hb.min_y ||
         mb.min_height >= hb.max_height || mb.max_height <= hb.min_height)) {

        std::cerr << "SimulationRunImpl::run: mission boundaries have no overlap with the hidden map"
                     " — check map offset vs mission boundaries\n";

        types::SimulationResult result;
        result.simulation_config = simulation_config_;
        result.mission_config = mission_config_;
        result.resolution_request_status =
            resolutionRequestStatus(simulation_config_, mission_config_);
        result.output_map_file = output_map_file_;
        result.output_map_config = output_map_->getMapConfig();
        result.mission_score = -1.0;

        common_types::MissionRunResult boundary_error;
        boundary_error.status = common_types::MissionRunStatus::Error;
        boundary_error.steps = 0;
        boundary_error.errors.push_back({
            "MISSION_BOUNDARY_INVALID",
            "mission boundaries have no overlap with the hidden map"
            " — check map offset vs mission boundaries"});

        result.mission_results = {boundary_error};
        return result;
    }

    const common_types::MissionRunResult mission_result = mission_control_->runMission();

    const common_types::MapConfig output_map_config = output_map_->getMapConfig();

    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.resolution_request_status =
        resolutionRequestStatus(simulation_config_, mission_config_);
    result.mission_results = {mission_result};
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_config;

    // fix from project 2
    if (mission_result.status == common_types::MissionRunStatus::Error) {
    result.mission_score = -1.0;
    } else {
        const std::vector<double> scores =
            MapsComparison::compare(*hidden_map_, {output_map_.get()});

        result.mission_score = scores[0];
    }

    return result;
}

} // namespace simulator