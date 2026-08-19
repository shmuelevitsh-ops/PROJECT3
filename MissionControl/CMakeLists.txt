add_library(MissionControl_322889890_315113738 SHARED
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/ScanResultToVoxels.cpp
)

set_target_properties(MissionControl_322889890_315113738 PROPERTIES
    PREFIX ""
)

target_include_directories(MissionControl_322889890_315113738 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

target_link_libraries(MissionControl_322889890_315113738 PRIVATE
    common::common
)

drone_warnings(MissionControl_322889890_315113738)


# for tests
find_package(GTest CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)

add_executable(mission_control_component_test
    tests/component/drone_control_test.cpp
    tests/component/mission_control_test.cpp
    tests/component/scan_result_to_voxels_test.cpp
    tests/component/support/RegistrationStub.cpp
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/ScanResultToVoxels.cpp
    # Test-only: scan_result_to_voxels_test.cpp needs a real mutable IMap3D to observe
    # voxel writes across a beam (a mock can't stand in the way it can for
    # interaction-only tests). Mirrors the Algorithm/tests/component -> Map3DImpl
    # test-only link (PROJ2_TESTS_PLAN.md §6); introduces no production dependency --
    # MissionControl_322889890_315113738 above is unaffected.
    ${CMAKE_CURRENT_SOURCE_DIR}/../Simulator/src/Map3DImpl.cpp
)

target_include_directories(mission_control_component_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/component
    ${CMAKE_CURRENT_SOURCE_DIR}/../Simulator/include
)

target_link_libraries(mission_control_component_test PRIVATE
    common::common
    TinyNPY::TinyNPY
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
)

drone_warnings(mission_control_component_test)