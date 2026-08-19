#pragma once

#include <Common/IGPS.h>

namespace simulator {

class MockGPS final : public common::IGPS {
public:
    MockGPS(
        common::Position3D position,
        common::Orientation heading,
        common::PhysicalLength resolution);

    [[nodiscard]] common::Position3D position() const override;
    [[nodiscard]] common::Orientation heading() const override;

    void setPosition(common::Position3D position);
    void setHeading(common::Orientation heading);

private:
    common::Position3D position_{};
    common::Orientation heading_{};
    common::PhysicalLength resolution_{};
};

} // namespace simulator
