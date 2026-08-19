// Test-support stub, not a migrated test. simulator_integration_test compiles
// MappingAlgorithmImpl.cpp and MissionControlImpl.cpp directly (real cross-module
// linkage, PROJ2_TESTS_EXECUTION_PLAN.md Phase 6 -- mirrors the test-only
// algorithm_component_test/mission_control_component_test -> Map3DImpl link decided
// in Phase 3, see PROJ2_TESTS_PLAN.md §6), so both files' REGISTER_MAPPING_ALGORITHM /
// REGISTER_MISSION_CONTROL macros run at static-init time (Common/MappingAlgorithmRegistration.h,
// Common/MissionControlRegistration.h), which need definitions of
// common::MappingAlgorithmRegistration's / common::MissionControlRegistration's constructors.
// The real definitions (Simulator/src/MappingAlgorithmRegistration.cpp,
// Simulator/src/MissionControlRegistration.cpp) register into simulator::Registrar's
// dlopen-plugin machinery -- irrelevant here, since the tests that need the real
// algorithm/mission-control behavior construct their own MappingAlgorithmFactory/
// MissionControlFactory locally instead of going through the plugin load path. This stub
// discards the factory instead of registering it anywhere, exactly like the per-module
// component-test stubs it mirrors.
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory) {}

MissionControlRegistration::MissionControlRegistration(MissionControlFactory) {}

} // namespace common
