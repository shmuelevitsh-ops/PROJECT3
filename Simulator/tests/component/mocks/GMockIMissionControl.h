#pragma once

#include <Common/IMissionControl.h>

#include <gmock/gmock.h>

namespace test {

class GMockIMissionControl : public common::IMissionControl {
public:
    MOCK_METHOD(common::types::MissionRunResult, runMission, (), (override));
};

} // namespace test
