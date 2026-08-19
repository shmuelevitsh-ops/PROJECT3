// Test-support stub, not a migrated test. MappingAlgorithmImpl.cpp runs
// REGISTER_MAPPING_ALGORITHM at static-init time (Common/MappingAlgorithmRegistration.h),
// which needs a definition of common::MappingAlgorithmRegistration's constructor. The real
// definition (Simulator/src/MappingAlgorithmRegistration.cpp) registers into
// simulator::Registrar's dlopen-plugin machinery -- a Simulator-only concern this target has
// no business depending on, since it compiles MappingAlgorithmImpl.cpp directly rather than
// through the plugin load path (PROJ2_TESTS_PLAN.md §6). This stub discards the factory
// instead of registering it anywhere.
#include <Common/MappingAlgorithmRegistration.h>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory) {}

} // namespace common
