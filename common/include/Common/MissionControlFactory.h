#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <functional>
#include <memory>

namespace common {

struct MissionControlDependencies {
    const types::MissionConfigData& mission_config;
    const types::DroneConfigData& drone_config; // mission will create its own drone controller
    ILidar& lidar;
    IGPS& gps;
    IDroneMovement& movement;
    IMutableMap3D& output_map;
    IMappingAlgorithm& mapping_algorithm;
    std::filesystem::path output_map_file;
    bool verbose = false;
};

using MissionControlFactory =
    std::function<std::unique_ptr<IMissionControl>(MissionControlDependencies)>;

} // namespace common
