#include <Simulator/MockGPS.h>

namespace simulator {

MockGPS::MockGPS(
    common::Position3D position,
    common::Orientation heading,
    common::PhysicalLength resolution)
    : position_(position),
      heading_(heading),
      resolution_(resolution) {}

common::Position3D MockGPS::position() const {
    return position_;
}

common::Orientation MockGPS::heading() const {
    return heading_;
}

void MockGPS::setPosition(common::Position3D position) {
    position_ = position;
}

void MockGPS::setHeading(common::Orientation heading) {
    heading_ = heading;
}

} // namespace simulator
