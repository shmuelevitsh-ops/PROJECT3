#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <MissionControl/IDroneControl.h>

#include <cstddef>
#include <optional>

namespace MissionControl_322889890_315113738 {

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    common::types::DroneConfigData drone_;
    common::types::MissionConfigData mission_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    std::optional<common::types::LidarScanResult> latest_scan_{};
};

} // namespace MissionControl_322889890_315113738