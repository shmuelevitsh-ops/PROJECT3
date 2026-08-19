#pragma once

#include <Common/ILidar.h>

#include <gmock/gmock.h>

namespace test {

class GMockILidar : public common::ILidar {
public:
    MOCK_METHOD(common::types::LidarScanResult, scan, (common::Orientation scan_orientation), (const, override));
    MOCK_METHOD(common::types::LidarConfigData, config, (), (const, override));
};

} // namespace test
