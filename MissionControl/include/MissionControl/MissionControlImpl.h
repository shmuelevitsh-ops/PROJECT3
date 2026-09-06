#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/DroneControlImpl.h>

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
    // Concrete type (not IDroneControl) so runMission() can query lastStepLog() for verbose
    // output after each step().
    std::unique_ptr<DroneControlImpl> drone_control_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
};

} // namespace mission_control_322889890_315113738