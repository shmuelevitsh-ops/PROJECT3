#pragma once

#include <Common/IMap3D.h>

#include <vector>

namespace simulator {

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const common::IMap3D& origin,
                                                    const std::vector<common::IMap3D*> targets);
};

} // namespace simulator