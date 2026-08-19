#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationRunImpl.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace simulator {

// Resolves the actual resolution output_map is built with: always
// simulation.map_resolution -- the hidden map's own resolution, and the only
// value MapsComparison can score the output map against. Whether the
// mission's requested resolution (gps_resolution * factor) happens to match
// it (ACCEPTED) or not (IGNORED / IGNORED TOO SMALL) is decided separately
// by SimulationRunImpl::resolutionRequestStatus(). Declared in
// SimulationRunFactoryImpl.h and shared with SimulationManager's per-run
// error path so a run that fails before its output map is built can still
// report a meaningful resolution_cm.
common::PhysicalLength outputMapResolution(const types::SimulationConfigData& simulation) {
    return simulation.map_resolution;
}

namespace {

constexpr const char* kOutputMapFileName = "map_output.npy";

// Builds hidden_map's MapConfig from the loaded array's shape, keeping
// resolution/offset consistent with `simulation` regardless of whether the
// shape came from a real load or the load-failure fallback below.
common::types::MapConfig hiddenMapConfig(const types::SimulationConfigData& simulation, const NpyArray::shape_t& shape) {
    const double resolution_cm = simulation.map_resolution.force_numerical_value_in(common::cm);
    const double extent_x_voxels = static_cast<double>(shape.size() > 0 ? shape[0] : 1);
    const double extent_y_voxels = static_cast<double>(shape.size() > 1 ? shape[1] : 1);
    const double extent_z_voxels = static_cast<double>(shape.size() > 2 ? shape[2] : 1);

    return common::types::MapConfig{
        common::types::MappingBounds{
            simulation.map_offset.x, simulation.map_offset.x + extent_x_voxels * resolution_cm * common::x_extent[common::cm],
            simulation.map_offset.y, simulation.map_offset.y + extent_y_voxels * resolution_cm * common::y_extent[common::cm],
            simulation.map_offset.z, simulation.map_offset.z + extent_z_voxels * resolution_cm * common::z_extent[common::cm]},
        simulation.map_offset,
        simulation.map_resolution};
}

// Loads the hidden map from simulation.map_filename.
//
// On load failure, logs to std::cerr and falls back to an empty/Unmapped
// array so the run can still complete (scoring 0 against an empty map rather
// than crashing the composition).
std::unique_ptr<Map3DImpl> loadHiddenMap(const types::SimulationConfigData& simulation) {
    auto array = std::make_shared<NpyArray>();
    const LPCSTR load_error = array->LoadNPY(simulation.map_filename.string());
    if (load_error != nullptr) {
        std::cerr << "SimulationRunFactoryImpl::create: failed to load hidden map '"
                  << simulation.map_filename.string() << "': " << load_error << '\n';
        array = std::make_shared<NpyArray>();
    }
    return std::make_unique<Map3DImpl>(array, hiddenMapConfig(simulation, array->Shape()));
}

// Builds output_map's MapConfig from mission.mission_bounds (offset/boundaries) and
// simulation.map_resolution (resolution) -- the mapping algorithm must stay unaware of the hidden
// map's geometry beyond that shared resolution. offset = boundaries.min on every axis guarantees
// no negative array indices for any in-bounds position, which Map3DImpl's fresh-map allocation
// requires.
common::types::MapConfig outputMapConfig(const types::SimulationConfigData& simulation,
                                         const common::types::MissionConfigData& mission) {
    return common::types::MapConfig{
        mission.mission_bounds,
        common::Position3D{mission.mission_bounds.min_x, mission.mission_bounds.min_y, mission.mission_bounds.min_height},
        outputMapResolution(simulation)};
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(common::MappingAlgorithmFactory mapping_algorithm_factory,
                                                    common::MissionControlFactory mission_control_factory,
                                                    bool verbose)
    : mapping_algorithm_factory_(std::move(mapping_algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      verbose_(verbose) {}

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
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    auto mapping_algorithm = mapping_algorithm_factory_(
        common::MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    // `output_path` is already this run's dedicated leaf directory (named by the caller --
    // SimulationManager -- from the simulation/mission/drone/lidar config names), so a fixed
    // filename is sufficient; uniqueness comes from the directory, not the filename.
    std::filesystem::create_directories(output_path);
    const std::filesystem::path output_map_file = output_path / kOutputMapFileName;

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
