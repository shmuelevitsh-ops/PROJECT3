#pragma once

#include <Common/Types.h>

namespace common {

class ILidar {
public:
    virtual ~ILidar() = default;
    [[nodiscard]] virtual types::LidarScanResult scan(Orientation scan_orientation) const = 0;
    [[nodiscard]] virtual types::LidarConfigData config() const = 0;
};

} // namespace common
