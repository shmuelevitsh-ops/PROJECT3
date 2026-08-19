#include <Simulator/Registrar.h>

#include <Simulator/ConfigLoader.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationException.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

std::shared_ptr<NpyArray> loadNpyOrThrow(const std::filesystem::path& path) {
    auto array = std::make_shared<NpyArray>();
    const LPCSTR error = array->LoadNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error("failed to load '" + path.string() + "': " + error);
    }
    return array;
}

// A fresh, empty (all-Unmapped) map anchored on the mission's own spatial
// bounds and GPS resolution. There is no SimulationConfigData in this test
// to derive a separately-resolved hidden/ground-truth map from, so the
// mission-control test below intentionally builds two such maps sharing one
// coordinate space: one as the ground truth MockLidar scans, one as the
// mission's own output_map.
std::unique_ptr<simulator::Map3DImpl> freshMissionMap(const common::types::MissionConfigData& mission) {
    const common::types::MapConfig config{
        mission.mission_bounds,
        Position3D{mission.mission_bounds.min_x, mission.mission_bounds.min_y, mission.mission_bounds.min_height},
        mission.gps_resolution};
    return std::make_unique<simulator::Map3DImpl>(std::make_shared<NpyArray>(), config);
}

} // namespace

// This suite's three real checks -- rejecting a mismatched-type load, creating and using a mapping
// algorithm, creating and running a mission control -- all live in one TEST() rather than three
// separate TEST_F()s sharing static fixture state. They previously depended on running in a fixed
// order (mismatched-type-rejection first, since it performed the only loads and stashed the
// resulting factories in static members for the other two to reuse), but GTest's --gtest_shuffle
// permutes test order *within* a suite as well as across suites, so nothing guaranteed that order.
// Putting all three in one function makes the order a property of ordinary sequential statement
// execution instead of GTest scheduling, so it can't be shuffled apart. It also removes the need
// for static fixture members (and the TearDownTestSuite that had to release them before Registrar's
// own static teardown): the local factories below are destroyed when this function returns, well
// before process exit.
//
// Root cause of why the loads must happen exactly once, in this test, at all (confirmed via
// LD_DEBUG=files and readelf while building this suite): Algorithm_..so contains STB_GNU_UNIQUE
// symbols -- an unavoidable byproduct of instantiating std:: templates (unique_ptr, shared_ptr, ...)
// in any nontrivial C++ shared library -- which makes glibc's dynamic linker mark the object
// NODELETE on its very first dlopen. Once NODELETE, dlclose() never actually unmaps it, so no later
// dlopen on that same path ever re-runs its global constructors again for the rest of the process,
// regardless of Registrar's own resize() correctness. That makes "reload the same already-touched
// .so correctly after a failed mismatched load" untestable in a single process: this test therefore
// only verifies that a wrong-type load throws PLUGIN_NOT_REGISTERED with the right code, using
// ALGORITHM_PLUGIN_PATH's one-and-only legitimate load (done first) to obtain the mapping algorithm
// factory the rest of this test reuses, and only then probing the mismatched load.
TEST(RegistrationEndToEnd, RejectsMismatchedTypeThenLoadsAndCreatesBoth) {
    auto& registrar = simulator::Registrar::instance();

    // The one-and-only legitimate load of ALGORITHM_PLUGIN_PATH in this
    // process; must come before the mismatched probe below (see NODELETE
    // note above).
    const common::MappingAlgorithmFactory mapping_algorithm_factory =
        registrar.loadMappingAlgorithm(ALGORITHM_PLUGIN_PATH);
    ASSERT_TRUE(mapping_algorithm_factory);

    try {
        (void)registrar.loadMissionControl(ALGORITHM_PLUGIN_PATH);
        FAIL() << "loadMissionControl(ALGORITHM_PLUGIN_PATH) should have thrown";
    } catch (const simulator::SimulationException& e) {
        EXPECT_EQ(e.code(), "PLUGIN_NOT_REGISTERED");
    }

    const common::MissionControlFactory mission_control_factory =
        registrar.loadMissionControl(MISSION_CONTROL_PLUGIN_PATH);
    ASSERT_TRUE(mission_control_factory);

    const std::filesystem::path inputs = TEST_INPUTS_DIR;
    const common::types::MissionConfigData mission =
        simulator::parseMissionConfig(inputs / "mission/small_mission_room.yaml");
    const common::types::LidarConfigData lidar =
        simulator::parseLidarConfig(inputs / "lidar/lidar_short.yaml");
    const common::types::DroneConfigData drone =
        simulator::parseDroneConfig(inputs / "drone/drone_small.yaml");

    // --- Create and use the mapping algorithm ---
    {
        const auto map = std::make_unique<simulator::Map3DImpl>(loadNpyOrThrow(inputs / "map/scenario_small.npy"));

        const auto instance =
            mapping_algorithm_factory(common::MappingAlgorithmDependencies{mission, lidar, drone, *map});
        ASSERT_TRUE(instance);

        // scenario_small.npy is 20x20x20 voxels; Map3DImpl infers a 1cm/voxel
        // default config from its shape when no explicit MapConfig is given, so
        // (10cm, 10cm, 10cm) sits at the center of the map.
        const common::types::DroneState state{
            Position3D{10.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]}, Orientation{}, 0};
        EXPECT_NO_THROW(instance->nextStep(state, nullptr));
    }

    // --- Create and run the mission control ---
    {
        const auto output_map = freshMissionMap(mission);
        const auto hidden_map = freshMissionMap(mission);

        const auto mapping_algorithm =
            mapping_algorithm_factory(common::MappingAlgorithmDependencies{mission, lidar, drone, *output_map});
        ASSERT_TRUE(mapping_algorithm);

        const double center_x_cm = (mission.mission_bounds.min_x.force_numerical_value_in(cm) +
                                     mission.mission_bounds.max_x.force_numerical_value_in(cm)) /
                                    2.0;
        const double center_y_cm = (mission.mission_bounds.min_y.force_numerical_value_in(cm) +
                                     mission.mission_bounds.max_y.force_numerical_value_in(cm)) /
                                    2.0;
        const double center_z_cm = (mission.mission_bounds.min_height.force_numerical_value_in(cm) +
                                     mission.mission_bounds.max_height.force_numerical_value_in(cm)) /
                                    2.0;
        const Position3D center{center_x_cm * x_extent[cm], center_y_cm * y_extent[cm], center_z_cm * z_extent[cm]};

        simulator::MockGPS gps(center, Orientation{}, mission.gps_resolution);
        simulator::MockMovement movement(gps);
        simulator::MockLidar lidar_sensor(lidar, *hidden_map, gps);

        const std::filesystem::path output_dir = std::filesystem::temp_directory_path() / "simulator_registration_test";
        std::filesystem::create_directories(output_dir);
        const std::filesystem::path output_map_file = output_dir / "map_output.npy";
        std::filesystem::remove(output_map_file);

        const auto instance = mission_control_factory(common::MissionControlDependencies{
            .mission_config = mission,
            .drone_config = drone,
            .lidar = lidar_sensor,
            .gps = gps,
            .movement = movement,
            .output_map = *output_map,
            .mapping_algorithm = *mapping_algorithm,
            .output_map_file = output_map_file,
            .verbose = false});
        ASSERT_TRUE(instance);

        EXPECT_NO_THROW(instance->runMission());
        EXPECT_TRUE(std::filesystem::exists(output_map_file));
    }
}

TEST(RegistrationEndToEnd, UnloadsAtProcessEnd) {
    // No per-test assertion beyond the successful loads already exercised
    // above; the real check is that this whole binary exits 0 without a
    // use-after-dlclose crash during Registrar's static teardown (which
    // must destroy factories_ before libraries_ -- see Registrar::~Registrar).
    SUCCEED();
}
