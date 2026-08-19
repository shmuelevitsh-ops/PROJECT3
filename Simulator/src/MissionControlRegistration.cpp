#include <Common/MissionControlRegistration.h>

#include <Simulator/Registrar.h>

#include <utility>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator::Registrar::instance().addMissionControl(std::move(factory));
}

} // namespace common
