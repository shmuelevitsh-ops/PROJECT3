#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace simulator {

class MockLidar final : public common::ILidar {
public:
    MockLidar(
        common::types::LidarConfigData config,
        const common::IMap3D& map,
        const common::IGPS& gps);

    [[nodiscard]] common::types::LidarScanResult scan(
        common::Orientation scan_orientation) const override;

    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    [[nodiscard]] common::PhysicalLength traceBeam(
        const common::Orientation& beam) const;

    common::types::LidarConfigData config_;
    const common::IMap3D& map_;
    const common::IGPS& gps_;
};

} // namespace simulator
