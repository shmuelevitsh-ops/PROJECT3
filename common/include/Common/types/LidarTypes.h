#pragma once

#include <Common/Units.h>

#include <cstddef>
#include <vector>

namespace common::types {

struct LidarConfigData {
    PhysicalLength z_min{};
    PhysicalLength z_max{};
    PhysicalLength d{};
    std::size_t fov_circles = 0;
};

struct LidarHit {
    PhysicalLength distance{};
    Orientation angle{};
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace common::types
