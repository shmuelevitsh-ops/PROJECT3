#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationRunImpl.h>

#include <memory>
#include <string>
#include <utility>

namespace simulator {

// In our implementation, the output map is built with the same resolution
// as the hidden map: simulation.map_resolution.
// The mission's requested resolution (gps_resolution * factor) is checked
// separately by SimulationRunImpl::resolutionRequestStatus(), which decides
// whether that request is ACCEPTED or IGNORED.
common::PhysicalLength outputMapResolution(const types::SimulationConfigData& simulation) {
    return simulation.map_resolution;
}

namespace {

constexpr const char* kOutputMapFileName = "map_output.npy";

// Builds hidden_map's MapConfig
// Combines the NPY shape with the simulation's resolution and offset
// to build the hidden map's physical bounds.
common::types::MapConfig hiddenMapConfig(const types::SimulationConfigData& simulation, const NpyArray::shape_t& shape) {
    const double resolution_cm = simulation.map_resolution.force_numerical_value_in(common::cm);
    const double extent_x_voxels = static_cast<double>(shape.size() > 0 ? shape[0] : 1);
    const double extent_y_voxels = static_cast<double>(shape.size() > 1 ? shape[1] : 1);
    const double extent_z_voxels = static_cast<double>(shape.size() > 2 ? shape[2] : 1);

    return common::types::MapConfig{
        common::types::MappingBounds{
            -simulation.map_offset.x, extent_x_voxels * resolution_cm * common::x_extent[common::cm] - simulation.map_offset.x,
            -simulation.map_offset.y, extent_y_voxels * resolution_cm * common::y_extent[common::cm] - simulation.map_offset.y,
            -simulation.map_offset.z, extent_z_voxels * resolution_cm * common::z_extent[common::cm] - simulation.map_offset.z},
        simulation.map_offset,
        simulation.map_resolution};
}

// Loads the hidden map from simulation.map_filename.
// If loading fails, throws so SimulationManager can report the run as -1/Error.
std::unique_ptr<Map3DImpl> loadHiddenMap(const types::SimulationConfigData& simulation) {
    auto array = std::make_shared<NpyArray>();
    const LPCSTR load_error = array->LoadNPY(simulation.map_filename.string());
    if (load_error != nullptr) {
        throw SimulationException(
            "MAP_LOAD_FAILED",
            "SimulationRunFactoryImpl::create: failed to load hidden map '" +
                simulation.map_filename.string() + "': " + load_error);
    }
    return std::make_unique<Map3DImpl>(array, hiddenMapConfig(simulation, array->Shape()));
}

// Builds the output map from the mission bounds.
// map_local = mission_relative + offset, so voxel index 0 sits at mission_relative = -offset;
// its offset is therefore the negated minimum mission corner, putting that corner at index 0.
common::types::MapConfig outputMapConfig(const types::SimulationConfigData& simulation,
                                         const common::types::MissionConfigData& mission) {
    return common::types::MapConfig{
        mission.mission_bounds,
        common::Position3D{-mission.mission_bounds.min_x, -mission.mission_bounds.min_y, -mission.mission_bounds.min_height},
        outputMapResolution(simulation)};
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(common::MappingAlgorithmFactory mapping_algorithm_factory,
                                                    common::MissionControlFactory mission_control_factory,
                                                    bool verbose)
    : mapping_algorithm_factory_(std::move(mapping_algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      verbose_(verbose) {}


// Builds and connects all runtime components required for one simulation run,
// then returns a ready-to-run ISimulationRun.
// Project 3 components such as MappingAlgorithm and MissionControl are created
// through injected factories, keeping Simulator independent of their implementations.
std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const common::types::MissionConfigData& mission,
                                 const common::types::DroneConfigData& drone,
                                 const common::types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    auto hidden_map = loadHiddenMap(simulation);
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), outputMapConfig(simulation, mission));

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        common::Orientation{simulation.initial_angle, 0.0 * common::altitude_angle[common::deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    
    // Project 3: create the mapping algorithm through the injected factory,
    // keeping Simulator independent of the concrete algorithm implementation.
    auto mapping_algorithm = mapping_algorithm_factory_(
        common::MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    // Each run has its own output directory, so a fixed map filename is good and unique.
    std::filesystem::create_directories(output_path);
    const std::filesystem::path output_map_file = output_path / kOutputMapFileName;

    // Project 3: create MissionControl through the injected factory,
    // keeping Simulator independent of the concrete MissionControl implementation.
    auto mission_control = mission_control_factory_(common::MissionControlDependencies{
        mission, drone, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm, output_map_file, verbose_});

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace simulator
