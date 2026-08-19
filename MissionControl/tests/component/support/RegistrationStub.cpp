// Test-support stub, not a migrated test. MissionControlImpl.cpp runs
// REGISTER_MISSION_CONTROL at static-init time (Common/MissionControlRegistration.h), which
// needs a definition of common::MissionControlRegistration's constructor. The real definition
// (Simulator/src/MissionControlRegistration.cpp) registers into simulator::Registrar's
// dlopen-plugin machinery -- a Simulator-only concern this target has no business depending on,
// since it compiles MissionControlImpl.cpp directly rather than through the plugin load path
// (PROJ2_TESTS_PLAN.md §6). This stub discards the factory instead of registering it anywhere.
#include <Common/MissionControlRegistration.h>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory) {}

} // namespace common
