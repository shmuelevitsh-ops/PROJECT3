#pragma once

#include <Common/IGPS.h>

#include <gmock/gmock.h>

namespace test {

class GMockIGPS : public common::IGPS {
public:
    MOCK_METHOD(common::Position3D, position, (), (const, override));
    MOCK_METHOD(common::Orientation, heading, (), (const, override));
};

} // namespace test
