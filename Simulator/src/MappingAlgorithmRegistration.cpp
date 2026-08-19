#include <Common/MappingAlgorithmRegistration.h>

#include <Simulator/Registrar.h>

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::Registrar::instance().addMappingAlgorithm(std::move(factory));
}

} // namespace common
