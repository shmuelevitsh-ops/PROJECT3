#pragma once

#include <Common/Types.h>

namespace common {

class IMissionControl {
public:
    virtual ~IMissionControl() = default;
    [[nodiscard]] virtual types::MissionRunResult runMission() = 0;
};

} // namespace common
