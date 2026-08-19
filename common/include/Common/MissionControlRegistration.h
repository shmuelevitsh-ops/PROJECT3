#pragma once

#include <Common/MissionControlFactory.h>

#include <utility>

namespace common {

struct MissionControlRegistration {
    explicit MissionControlRegistration(MissionControlFactory factory);
};

} // namespace common

#define REGISTER_MISSION_CONTROL(class_name)                                      \
    [[maybe_unused]] ::common::MissionControlRegistration register_me_##class_name{ \
        [](::common::MissionControlDependencies dependencies)                     \
            -> std::unique_ptr<::common::IMissionControl> {                       \
            return std::make_unique<class_name>(std::move(dependencies));          \
        }}
