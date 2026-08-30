#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/IDroneControl.h>

#include <filesystem>
#include <memory>

namespace mission_control_322889890_315113738 {

class MissionControlImpl final : public common::IMissionControl {
public:
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_;
    const common::IMutableMap3D& output_map_;
    std::unique_ptr<mission_control::IDroneControl> drone_control_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
};

} // namespace mission_control_322889890_315113738