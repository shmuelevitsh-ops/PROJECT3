// Test-only plugin .so, built solely so component_construction_test.cpp can exercise the
// Simulator's component-level failure path: a MissionControl that loads/registers successfully
// (REGISTER_MISSION_CONTROL runs fine) but whose factory throws while constructing the instance.
// Mirrors double_mapping_algorithm_test_plugin's build (declares only the REGISTER_* macro, links
// only common::common) -- dlopen'd at runtime by the spawned simulator_322889890_315113738
// binary, which supplies the real common::MissionControlRegistration constructor.
#include <Common/MissionControlRegistration.h>

#include <stdexcept>

namespace {

class ThrowingMissionControl : public common::IMissionControl {
public:
    explicit ThrowingMissionControl(common::MissionControlDependencies) {
        throw std::runtime_error("ThrowingMissionControl refuses to be constructed");
    }

    common::types::MissionRunResult runMission() override {
        return {};
    }
};

} // namespace

REGISTER_MISSION_CONTROL(ThrowingMissionControl);
