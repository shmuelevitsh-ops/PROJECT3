// Test-only plugin .so, built solely to exercise Registrar::loadMappingAlgorithm's
// more-than-one-registration rejection path (multiple_registration_test.cpp). Registers two
// distinct IMappingAlgorithm classes via REGISTER_MAPPING_ALGORITHM, so a single dlopen()
// of this library registers two mapping algorithm factories instead of the required one. Mirrors
// Algorithm_322889890_315113738's own build: only declares the REGISTER_MAPPING_ALGORITHM macro
// and links common::common, relying on the loading process (simulator_registration_test, built
// with ENABLE_EXPORTS ON and linking Simulator/src/MappingAlgorithmRegistration.cpp) to resolve
// common::MappingAlgorithmRegistration's constructor at dlopen time.
#include <Common/MappingAlgorithmRegistration.h>

namespace {

class DummyMappingAlgorithmA : public common::IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                                const common::types::LidarScanResult*) override {
        return {};
    }
};

class DummyMappingAlgorithmB : public common::IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                                const common::types::LidarScanResult*) override {
        return {};
    }
};

} // namespace

REGISTER_MAPPING_ALGORITHM(DummyMappingAlgorithmA);
REGISTER_MAPPING_ALGORITHM(DummyMappingAlgorithmB);
