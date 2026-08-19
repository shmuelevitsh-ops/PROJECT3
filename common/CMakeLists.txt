add_library(drone_common INTERFACE)
add_library(common::common ALIAS drone_common)

target_include_directories(drone_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_common INTERFACE mp-units::mp-units)
