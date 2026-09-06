#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/DroneControlImpl.h>

#include <filesystem>
#include <fstream>
#include <memory>

namespace mission_control_322889890_315113738 {

struct StepLoopOutcome {
    common::types::MissionRunStatus status = common::types::MissionRunStatus::MaxSteps;
    std::size_t steps = 0;
    std::string completion_message;
    std::vector<common::types::ErrorRef> errors;
};

class MissionControlImpl final : public common::IMissionControl {
public:
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    // Runs the step loop until completion, error termination, or max_steps is reached, updating
    // outcome/verbose_log as it goes.
    void runStepLoop(StepLoopOutcome& outcome, std::ofstream& verbose_log);

    common::types::MissionConfigData mission_;
    const common::IMutableMap3D& output_map_;
    // Concrete type (not IDroneControl) so runMission() can query lastStepLog() for verbose
    // output after each step().
    std::unique_ptr<DroneControlImpl> drone_control_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
};

} // namespace mission_control_322889890_315113738