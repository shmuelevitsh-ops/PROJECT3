#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationRunImpl.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace simulator {

common::PhysicalLength outputMapResolution(const types::SimulationConfigData& simulation) {
    return simulation.map_resolution;
}

namespace {

// Builds a unique, traceable output map filename from the leaf output directory's path
// segments. SimulationManager encodes the evaluated component, simulation, mission, drone
// and lidar identity into output_path's directory names (component/simulations/sim/mission/
// drone__lidar); this reuses that same identity for the filename itself, so a map file stays
// unique and traceable even if it is ever copied out of its directory. The fixed "simulations"
// segment carries no identifying information, so it is dropped; only the last four remaining
// segments are kept.
std::string uniqueOutputMapFileName(const std::filesystem::path& output_path) {
    std::vector<std::string> segments;
    for (const auto& part : output_path) {
        const std::string segment = part.string();
        if (segment.empty() || segment == "/" || segment == "simulations") {
            continue;
        }
        segments.push_back(segment);
    }

    const std::size_t keep = std::min<std::size_t>(4, segments.size());
    std::string name;
    for (std::size_t i = segments.size() - keep; i < segments.size(); ++i) {
        if (!name.empty()) {
            name += "__";
        }
        name += segments[i];
    }
    return name + ".npy";
}

// Builds the hidden-map bounds from its shape, resolution and offset.
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

// Loads and validates the hidden map.
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

// Loads and validates the hidden map.
common::types::MapConfig outputMapConfig(const types::SimulationConfigData& simulation,
                                         const common::types::MissionConfigData& mission) {
    return common::types::MapConfig{
        mission.mission_bounds,
        common::Position3D{-mission.mission_bounds.min_x, -mission.mission_bounds.min_y, -mission.mission_bounds.min_height},
        outputMapResolution(simulation)};
}

// Invokes an injected component factory (MappingAlgorithm or MissionControl), translating any
// exception thrown while constructing the plugin instance into a ComponentConstructionException
// tagged with which factory it came from. This is what lets SimulationManager tell "this
// component's factory cannot build a runnable instance" apart from an ordinary per-run failure
// (e.g. a hidden map that fails to load), and tell the evaluated component's factory apart from
// the fixed/shared one's -- both can be thrown from within create(), but they mean different
// things to the caller.
template <typename Factory, typename Dependencies>
auto constructComponent(ComponentKind kind, const Factory& factory, Dependencies&& dependencies)
    -> decltype(factory(std::forward<Dependencies>(dependencies))) {
    try {
        return factory(std::forward<Dependencies>(dependencies));
    } catch (const std::exception& e) {
        throw ComponentConstructionException(kind, std::string("SimulationRunFactoryImpl::create: ") +
                                             toString(kind) +
                                             " factory failed to construct its component: " + e.what());
    }
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
        common::Orientation{simulation.initial_angle, 0.0 * common::altitude_angle[common::deg]});
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    
    // Project 3: create the mapping algorithm through the injected factory,
    // keeping Simulator independent of the concrete algorithm implementation.
    auto mapping_algorithm = constructComponent(
        ComponentKind::MappingAlgorithm, mapping_algorithm_factory_,
        common::MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    // Each run has its own output directory, but the filename itself is also made unique and
    // traceable (component/simulation/mission/drone/lidar), so it survives being copied out.
    std::filesystem::create_directories(output_path);
    const std::filesystem::path output_map_file = output_path / uniqueOutputMapFileName(output_path);

    // Project 3: create MissionControl through the injected factory,
    // keeping Simulator independent of the concrete MissionControl implementation.
    auto mission_control = constructComponent(
        ComponentKind::MissionControl, mission_control_factory_,
        common::MissionControlDependencies{mission, drone, *lidar_impl, *gps, *movement, *output_map,
                                           *mapping_algorithm, output_map_file, verbose_});

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
