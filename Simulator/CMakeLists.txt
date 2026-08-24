cmake_minimum_required(VERSION 3.15)

# Standalone configuration: only kicks in when this file is configured directly
# (Simulator/ built on its own), not when pulled in via add_subdirectory() from
# the root CMakeLists.txt, which already provides all of this.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # The vcpkg manifest (vcpkg.json) lives at the repo root, shared by every
    # subproject; point vcpkg at it so a standalone configure can still resolve
    # dependencies via CMAKE_TOOLCHAIN_FILE alone.
    if(NOT DEFINED VCPKG_MANIFEST_DIR)
        set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE STRING "")
    endif()

    project(Simulator LANGUAGES CXX)

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

    find_package(mp-units CONFIG REQUIRED)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common)
endif()

if(NOT COMMAND drone_warnings)
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

find_package(yaml-cpp REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(GTest CONFIG REQUIRED) # for tests

add_executable(simulator_322889890_315113738
    src/CerrContextGuard.cpp
    src/CliOptions.cpp
    src/ConfigLoader.cpp
    src/Map3DImpl.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MapsComparison.cpp
    src/MissionControlRegistration.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/Registrar.cpp
    src/SimulationManager.cpp
    src/SimulationOutputWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/SimulatorRunner.cpp
    src/drone_mapper_simulation_main.cpp
)

target_include_directories(simulator_322889890_315113738 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(simulator_322889890_315113738 PRIVATE
    common::common
    yaml-cpp::yaml-cpp
    TinyNPY::TinyNPY
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

set_target_properties(simulator_322889890_315113738 PROPERTIES
    ENABLE_EXPORTS ON
)

drone_warnings(simulator_322889890_315113738)


# for tests -- fixture plugin registering two mapping algorithms from one dlopen(), used only by
# multiple_registration_test.cpp to exercise Registrar's more-than-one-registration rejection path.
# Mirrors Algorithm_322889890_315113738's build (declares REGISTER_MAPPING_ALGORITHM, links only
# common::common): the loading test binary supplies MappingAlgorithmRegistration's constructor.
add_library(double_mapping_algorithm_test_plugin SHARED
    tests/registration/support/DoubleMappingAlgorithmPlugin.cpp
)

set_target_properties(double_mapping_algorithm_test_plugin PROPERTIES
    PREFIX ""
)

target_include_directories(double_mapping_algorithm_test_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(double_mapping_algorithm_test_plugin PRIVATE
    common::common
)

drone_warnings(double_mapping_algorithm_test_plugin)


# for tests
add_executable(simulator_registration_test
    tests/registration/registration_end_to_end_test.cpp
    tests/registration/multiple_registration_test.cpp
    tests/stage1_verify/stage1_verify_test.cpp
    tests/stage2_verify/stage2_verify_test.cpp
    tests/stage3_verify/stage3_verify_test.cpp
    tests/cerr_context_verify/cerr_context_verify_test.cpp
    tests/multithreading_verify/multithreading_verify_test.cpp
    src/CerrContextGuard.cpp
    src/CliOptions.cpp
    src/ConfigLoader.cpp
    src/Map3DImpl.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MapsComparison.cpp
    src/MissionControlRegistration.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/Registrar.cpp
    src/SimulationManager.cpp
    src/SimulationOutputWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/SimulatorRunner.cpp
)

target_include_directories(simulator_registration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(simulator_registration_test PRIVATE
    common::common
    yaml-cpp::yaml-cpp
    TinyNPY::TinyNPY
    Threads::Threads
    ${CMAKE_DL_LIBS}
    GTest::gtest
    GTest::gtest_main
)

target_compile_definitions(simulator_registration_test PRIVATE
    ALGORITHM_PLUGIN_PATH="$<TARGET_FILE:Algorithm_322889890_315113738>"
    MISSION_CONTROL_PLUGIN_PATH="$<TARGET_FILE:MissionControl_322889890_315113738>"
    DOUBLE_MAPPING_ALGORITHM_PLUGIN_PATH="$<TARGET_FILE:double_mapping_algorithm_test_plugin>"
    SIM_COMPOSE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../inputs/sim_compose.yaml"
    TEST_INPUTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../inputs"
)

set_target_properties(simulator_registration_test PROPERTIES
    ENABLE_EXPORTS ON
)

add_dependencies(simulator_registration_test
    Algorithm_322889890_315113738
    MissionControl_322889890_315113738
    double_mapping_algorithm_test_plugin
)

drone_warnings(simulator_registration_test)


# for tests -- sibling target hosting Simulator-owned component tests migrated from
# Project 2 (PROJ2_TESTS_PLAN.md §6 point 4), kept separate from the
# registration/verify-focused simulator_registration_test above.
add_executable(simulator_component_test
    tests/component/map3d_impl_test.cpp
    tests/component/maps_comparison_test.cpp
    tests/component/mock_lidar_test.cpp
    tests/component/simulation_manager_test.cpp
    tests/component/simulation_output_writer_test.cpp
    tests/component/simulation_run_factory_impl_test.cpp
    tests/component/simulation_run_test.cpp
    src/CerrContextGuard.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/SimulationManager.cpp
    src/SimulationOutputWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_include_directories(simulator_component_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/component
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(simulator_component_test PRIVATE
    common::common
    yaml-cpp::yaml-cpp
    TinyNPY::TinyNPY
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
)

target_compile_definitions(simulator_component_test PRIVATE
    DATA_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/component/data_maps"
)

drone_warnings(simulator_component_test)


# for tests -- sibling target hosting the Integration/Internal/Audit tests migrated from
# Project 2 (PROJ2_TESTS_EXECUTION_PLAN.md Phase 6), kept separate from
# simulator_component_test above. Unlike that target, several tests here exercise the real
# cross-module pipeline in-process (real DroneControlImpl/MissionControlImpl from
# MissionControl, real MappingAlgorithmImpl from Algorithm) instead of mocking those layers
# -- a test-only exception mirroring algorithm_component_test's/mission_control_component_test's
# real Map3DImpl link (PROJ2_TESTS_PLAN.md §6): it introduces no production dependency between
# the three modules, and none of Algorithm_322889890_315113738 / MissionControl_322889890_315113738
# / simulator_322889890_315113738 are affected. The remaining tests here spawn the real
# simulator_322889890_315113738 binary as a subprocess, which itself dlopens the real plugin
# .so's at runtime -- hence the add_dependencies below.
#
# Must be run with its working directory set to tests/integration/ (matches Project 2's own
# convention, preserved as-is per Phase 6: internal_flow_test.cpp's
# RealAlgorithmFlowOnSmallKnownMapDoesNotCrash reads a fixture via the relative literal
# "data_maps/single_voxel_x4_y4_z4.npy", unchanged from the Project 2 original).
add_executable(simulator_integration_test
    tests/integration/internal_flow_test.cpp
    tests/integration/full_flow_test.cpp
    tests/integration/audit_errors_test.cpp
    tests/integration/support/RegistrationStub.cpp
    src/CerrContextGuard.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/src/DroneControlImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/src/MissionControlImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/src/ScanResultToVoxels.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Algorithm/src/MappingAlgorithmImpl.cpp
)

target_include_directories(simulator_integration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/component
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/common_mission_control/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Algorithm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(simulator_integration_test PRIVATE
    common::common
    yaml-cpp::yaml-cpp
    TinyNPY::TinyNPY
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
)

target_compile_definitions(simulator_integration_test PRIVATE
    DRONE_MAPPER_SIMULATION_BINARY="$<TARGET_FILE:simulator_322889890_315113738>"
    ALGORITHM_PLUGIN_PATH="$<TARGET_FILE:Algorithm_322889890_315113738>"
    MISSION_CONTROL_PLUGIN_PATH="$<TARGET_FILE:MissionControl_322889890_315113738>"
    TEST_INPUTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_inputs"
    AUDIT_INPUTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/audit_inputs"
    DATA_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/data_maps"
)

add_dependencies(simulator_integration_test
    simulator_322889890_315113738
    Algorithm_322889890_315113738
    MissionControl_322889890_315113738
)

drone_warnings(simulator_integration_test)