// Migrated from Project 2 (FILES PROJECT 2/tests/components/simulation_manager_test.cpp).
// Verified against the staff mutated_src harness before porting — see
// PROJ2_TESTS_PLAN.md §4. Structurally adapted for Project 3's DI/module layout (see
// §3/§7.2): SimulationCompositionData's drone/lidar fields are named drone_configs/
// lidar_configs (renamed from drones/lidars -- mechanical only). More substantially,
// SimulationConfigData/MissionConfigData no longer carry their own load_error field (moved
// to a Simulator-local ReferencedConfigFile{path, load_error} wrapper alongside each
// CompositionFilePaths entry, §3) -- SimulationManager::run()'s 2-arg overload always
// builds synthetic file paths with load_error unset (buildSyntheticFilePaths() in
// SimulationManager.cpp), so it can never exercise the load-error skip path at all
// anymore. The three "...LoadErrorSkipsFactoryAndScoresNegativeOne(...)" tests below
// therefore switch from setting a load_error field directly on the config (no longer
// possible) to calling the 3-arg run() overload with an explicit CompositionFilePaths
// whose relevant ReferencedConfigFile carries the load_error -- the load-error scenario and
// assertions are otherwise unchanged from Project 2. Every other test's intent, inputs, and
// assertions are preserved 1:1.

#include <Simulator/CerrContextGuard.h>
#include <Simulator/SimulationException.h>
#include <Simulator/SimulationManager.h>

#include "mocks/GMockISimulationRun.h"
#include "mocks/GMockISimulationRunFactory.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace common;
using namespace common::types;
using namespace simulator;
using namespace simulator::types;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

namespace {

// 2 simulation groups (2 missions, 1 mission) x 2 drones x 2 lidars = 12.
constexpr std::size_t kExpectedCreateCalls = 12;

SimulationCompositionData nestedComposition() {
    SimulationCompositionData composition;
    composition.composition_file = "compositions.yaml";
    composition.simulation_mission_groups = {
        {SimulationConfigData{}, {MissionConfigData{}, MissionConfigData{}}},
        {SimulationConfigData{}, {MissionConfigData{}}},
    };
    composition.drone_configs = {DroneConfigData{}, DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}, LidarConfigData{}};
    return composition;
}

} // namespace

TEST(SimulationManager, RunCallsFactoryCreateOncePerCartesianCombinationAcrossNestedGroups) {
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(kExpectedCreateCalls)
        .WillRepeatedly(Invoke([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                                  const LidarConfigData&, const std::filesystem::path&) {
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(nestedComposition(), "tests/component/test_output");

    EXPECT_EQ(report.runs.size(), kExpectedCreateCalls);
}

TEST(SimulationManager, RunReturnsResultsMatchingMockReturnValuesInOrder) {
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    int next_score = 0;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(kExpectedCreateCalls)
        .WillRepeatedly(Invoke([&next_score](const SimulationConfigData&, const MissionConfigData&,
                                             const DroneConfigData&, const LidarConfigData&,
                                             const std::filesystem::path&) {
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            SimulationResult result;
            result.mission_score = static_cast<double>(next_score++);
            EXPECT_CALL(*run, run()).WillOnce(Return(result));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(nestedComposition(), "tests/component/test_output");

    ASSERT_EQ(report.runs.size(), kExpectedCreateCalls);
    for (std::size_t i = 0; i < kExpectedCreateCalls; ++i) {
        EXPECT_EQ(report.runs[i].mission_score, static_cast<double>(i))
            << "Mismatch at index " << i << " — results are not in creation order.";
    }
}

TEST(SimulationManager, ConstructorThrowsIfRunFactoryIsNull) {
    EXPECT_THROW(SimulationManager manager(nullptr), std::invalid_argument);
}

// New — closes a Project 2 mutation-coverage gap (MAN27, see
// PROJ2_TESTS_PLAN.md §8.2). score_range.min is a fixed part of the spec's
// {min:0, max:100} contract -- it must stay 0 even when the batch being
// summarized contains an error-scored (-1.0) run alongside a normal one.
TEST(SimulationManager, ScoreRangeMinStaysZeroEvenWhenBatchContainsErrorRuns) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {
        {SimulationConfigData{}, {MissionConfigData{}, MissionConfigData{}}},
    };
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    bool first_call = true;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly(Invoke([&first_call](const SimulationConfigData&, const MissionConfigData&,
                                             const DroneConfigData&, const LidarConfigData&,
                                             const std::filesystem::path&) {
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            SimulationResult result;
            result.mission_score = first_call ? 42.0 : -1.0;
            first_call = false;
            EXPECT_CALL(*run, run()).WillOnce(Return(result));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(
        composition, "tests/component/test_output/simulation_manager_test/score_range_min");

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0)
        << "score_range.min must stay fixed at 0 even when the batch contains an error-scored run";
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
}

TEST(SimulationManager, PopulatesFixedMetadataAndFreshUtcTimestamp) {
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([](const SimulationConfigData&,
                                                                     const MissionConfigData&,
                                                                     const DroneConfigData&,
                                                                     const LidarConfigData&,
                                                                     const std::filesystem::path&) {
        auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
        return std::unique_ptr<ISimulationRun>(std::move(run));
    }));

    const std::time_t before = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(nestedComposition(), "tests/component/test_output");
    const std::time_t after = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);

    static const std::regex kIso8601Utc(R"(^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$)");
    EXPECT_TRUE(std::regex_match(report.generated_at_utc, kIso8601Utc))
        << "generated_at_utc = " << report.generated_at_utc;

    std::tm parsed_tm{};
    std::istringstream(report.generated_at_utc) >> std::get_time(&parsed_tm, "%Y-%m-%dT%H:%M:%SZ");
    const std::time_t parsed_time = timegm(&parsed_tm);
    EXPECT_GE(parsed_time, before);
    EXPECT_LE(parsed_time, after);
}

TEST(SimulationManager, ThreeArgRunNamesOutputDirectoriesFromRealConfigFileStems) {
    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sims/sim_a.yaml"},
         {ReferencedConfigFile{"missions/mission_x.yaml"}, ReferencedConfigFile{"missions/mission_y.yaml"}}},
        {ReferencedConfigFile{"sims/sim_b.yaml"}, {ReferencedConfigFile{"missions/mission_z.yaml"}}},
    };
    file_paths.drone_paths = {"drones/drone_one.yaml", "drones/drone_two.yaml"};
    file_paths.lidar_paths = {"lidars/lidar_one.yaml", "lidars/lidar_two.yaml"};

    std::vector<std::filesystem::path> observed_output_paths;
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(kExpectedCreateCalls)
        .WillRepeatedly(Invoke([&observed_output_paths](const SimulationConfigData&, const MissionConfigData&,
                                                         const DroneConfigData&, const LidarConfigData&,
                                                         const std::filesystem::path& output_path) {
            observed_output_paths.push_back(output_path);
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    const std::filesystem::path output_root = "tests/component/test_output/simulation_manager_test/path_shape";
    std::filesystem::remove_all(output_root);

    SimulationManager manager(std::move(factory));
    (void)manager.run(nestedComposition(), output_root, file_paths);

    ASSERT_EQ(observed_output_paths.size(), kExpectedCreateCalls);
    // Loop order: sim_a/mission_x then sim_a/mission_y then sim_b/mission_z, each x drone_one
    // then drone_two, each x lidar_one then lidar_two -- matches SimulationManager::run()'s fixed
    // nested-loop order (documented and relied upon elsewhere, e.g. SimulationOutputWriter).
    EXPECT_EQ(observed_output_paths[0],
             output_root / "simulations" / "sim_a" / "mission_x" / "drone_one__lidar_one");
    // Indices 1 and 2 are the ones that actually distinguish "drone outer, lidar inner" from a
    // transposed "lidar outer, drone inner" loop order -- indices 0 and 3 are the cartesian
    // product's diagonal entries, which coincide under either loop order for a 2x2 drone/lidar
    // grid and so cannot catch a swapped loop nesting on their own.
    EXPECT_EQ(observed_output_paths[1],
             output_root / "simulations" / "sim_a" / "mission_x" / "drone_one__lidar_two")
        << "drone_index must be the outer loop relative to lidar_index: with drone_one still "
           "selected, lidar_index must already have advanced to lidar_two at index 1, not "
           "drone_index advancing to drone_two while lidar resets";
    EXPECT_EQ(observed_output_paths[2],
             output_root / "simulations" / "sim_a" / "mission_x" / "drone_two__lidar_one")
        << "after lidar_index exhausts its range for drone_one, drone_index must advance to "
           "drone_two and lidar_index must reset to lidar_one, since lidar is the inner loop";
    EXPECT_EQ(observed_output_paths[3],
             output_root / "simulations" / "sim_a" / "mission_x" / "drone_two__lidar_two");
    EXPECT_EQ(observed_output_paths[4],
             output_root / "simulations" / "sim_a" / "mission_y" / "drone_one__lidar_one");
    EXPECT_EQ(observed_output_paths[11],
             output_root / "simulations" / "sim_b" / "mission_z" / "drone_two__lidar_two");
}

TEST(SimulationManager, TwoArgRunFallsBackToSyntheticPositionalDirectoryNamesWithoutFilePaths) {
    std::vector<std::filesystem::path> observed_output_paths;
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _))
        .WillByDefault(Invoke([&observed_output_paths](const SimulationConfigData&, const MissionConfigData&,
                                                        const DroneConfigData&, const LidarConfigData&,
                                                        const std::filesystem::path& output_path) {
            observed_output_paths.push_back(output_path);
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    const std::filesystem::path output_root = "tests/component/test_output/simulation_manager_test/synthetic_names";
    std::filesystem::remove_all(output_root);

    SimulationManager manager(std::move(factory));
    (void)manager.run(nestedComposition(), output_root); // 2-arg overload has no real file paths.

    ASSERT_EQ(observed_output_paths.size(), kExpectedCreateCalls);
    EXPECT_EQ(observed_output_paths[0], output_root / "simulations" / "sim_0" / "mission_0_0" / "drone_0__lidar_0");
    // Every directory must still be distinct -- no two runs should collide even with placeholder names.
    const std::set<std::filesystem::path> unique_paths(observed_output_paths.begin(), observed_output_paths.end());
    EXPECT_EQ(unique_paths.size(), observed_output_paths.size());
}

TEST(SimulationManager, DisambiguatesOutputDirectoriesWhenTwoRunsWouldShareTheSameLeafName) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {
        {SimulationConfigData{}, {MissionConfigData{}, MissionConfigData{}}},
    };
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    CompositionFilePaths file_paths;
    // Both missions resolve to the same stem -- a real scenario when a composition references the
    // same mission file twice, or two files that happen to share a basename.
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sims/sim_a.yaml"},
         {ReferencedConfigFile{"missions/shared.yaml"}, ReferencedConfigFile{"missions/shared.yaml"}}},
    };
    file_paths.drone_paths = {"drones/drone_one.yaml"};
    file_paths.lidar_paths = {"lidars/lidar_one.yaml"};

    std::vector<std::filesystem::path> observed_output_paths;
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly(Invoke([&observed_output_paths](const SimulationConfigData&, const MissionConfigData&,
                                                         const DroneConfigData&, const LidarConfigData&,
                                                         const std::filesystem::path& output_path) {
            observed_output_paths.push_back(output_path);
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    const std::filesystem::path output_root = "tests/component/test_output/simulation_manager_test/collision";
    std::filesystem::remove_all(output_root);

    SimulationManager manager(std::move(factory));
    (void)manager.run(composition, output_root, file_paths);

    ASSERT_EQ(observed_output_paths.size(), 2u);
    const std::filesystem::path expected_first =
        output_root / "simulations" / "sim_a" / "shared" / "drone_one__lidar_one";
    EXPECT_EQ(observed_output_paths[0], expected_first);
    EXPECT_EQ(observed_output_paths[1], expected_first.string() + "__2")
        << "second run targeting the same leaf directory must be disambiguated, not silently"
           " overwrite the first run's map_output.npy";
}

TEST(SimulationManager, PrefixesCerrLinesWrittenDuringARunWithThatRunsContextAndLeavesOtherLinesAlone) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{SimulationConfigData{}, {MissionConfigData{}}}};
    composition.drone_configs = {DroneConfigData{}, DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {{ReferencedConfigFile{"sims/sim_a.yaml"}, {ReferencedConfigFile{"missions/mission_x.yaml"}}}};
    file_paths.drone_paths = {"drones/drone_one.yaml", "drones/drone_two.yaml"};
    file_paths.lidar_paths = {"lidars/lidar_one.yaml"};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    {
        ::testing::InSequence seq;
        EXPECT_CALL(*factory, create(_, _, _, _, _))
            .WillOnce(Invoke([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                               const LidarConfigData&, const std::filesystem::path&) {
                std::cerr << "first run message\n";
                auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
                ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
                return std::unique_ptr<ISimulationRun>(std::move(run));
            }));
        EXPECT_CALL(*factory, create(_, _, _, _, _))
            .WillOnce(Invoke([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                               const LidarConfigData&, const std::filesystem::path&) {
                std::cerr << "second run message\n";
                auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
                ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
                return std::unique_ptr<ISimulationRun>(std::move(run));
            }));
    }

    const std::filesystem::path output_root = "tests/component/test_output/simulation_manager_test/cerr_prefix";
    std::filesystem::remove_all(output_root);

    std::ostringstream captured;
    std::string log;
    {
        // CerrContextGuard (used internally by SimulationManager::run(), one per iteration) only
        // labels the current thread's prefix stack -- it never touches std::cerr itself. Prefixes
        // are actually applied by CerrSinkGuard's installed streambuf (Simulator/CerrContextGuard.h),
        // which must be constructed before anything writes to std::cerr for prefixing to occur at
        // all; plain std::cerr.rdbuf() swapping (the Project 2 original's approach, unneeded there
        // since P2 had no equivalent multi-threaded cerr-prefixing mechanism) captures output but
        // never triggers prefixing under Project 3's mechanism. Adapted here rather than a plain
        // rdbuf swap -- see PROJ2_TESTS_PLAN.md §7.1 provenance convention for behavior-affecting
        // (not just mechanical) changes.
        //
        // ConcurrentContextStreambuf (CerrContextGuard.cpp) buffers each thread's current line
        // until it sees '\n', then prefixes and flushes the whole line atomically -- required for
        // the non-interleaving guarantee under concurrent writes (see
        // CerrContextVerify.ConcurrentNonInterleavingAndCorrectlyAttributed). The Project 2
        // original's final write ("after every run", no trailing newline) relied on unprefixed
        // writes appearing immediately, which held for a plain rdbuf swap but never flushes under
        // this line-buffered mechanism -- not a production bug, just this mechanism's documented
        // contract (every other caller of it, including cerr_context_verify_test.cpp, always
        // writes complete lines). Added the trailing newline so the line actually flushes; the
        // assertion's intent (text written outside any run's guard scope stays unprefixed) is
        // unchanged.
        const CerrSinkGuard sink_guard(captured.rdbuf());
        std::cerr << "before any run\n";

        SimulationManager manager(std::move(factory));
        (void)manager.run(composition, output_root, file_paths);

        std::cerr << "after every run\n";
        log = captured.str();
    }
    EXPECT_NE(log.find("before any run\n"), std::string::npos)
        << "cerr output written before manager.run() must pass through unprefixed; captured log "
           "was:\n" << log;
    EXPECT_NE(log.find("after every run\n"), std::string::npos)
        << "cerr output written after manager.run() must pass through unprefixed; captured log "
           "was:\n" << log;
    EXPECT_NE(log.find("[sim=sim_a mission=mission_x drone=drone_one lidar=lidar_one] first run message\n"),
             std::string::npos)
        << "expected the first run's cerr line to be prefixed with its [sim=... mission=... "
           "drone=... lidar=...] context; captured log was:\n" << log;
    EXPECT_NE(log.find("[sim=sim_a mission=mission_x drone=drone_two lidar=lidar_one] second run message\n"),
             std::string::npos)
        << "expected the second run's cerr line to be prefixed with its own [sim=... mission=... "
           "drone=... lidar=...] context (drone=drone_two, distinct from the first run); captured "
           "log was:\n" << log;
}

// ── ConfigLoader load_error placeholders ─────────────────────────────────────
//
// A simulation_config/mission_config that failed to load/parse is represented by its
// CompositionFilePaths entry carrying a load_error (§3/§7.2 -- Project 2's SimulationConfigData/
// MissionConfigData carried load_error directly; Project 3 moved it out into
// ReferencedConfigFile, so these three tests drive the 3-arg run() overload with an explicit
// CompositionFilePaths instead of setting a field on the config). SimulationManager must
// recognize this *before* ever calling the factory (the group/mission's config is otherwise
// meaningless) and score it -1 directly. A bug here (e.g. checking the wrong field, or calling
// the factory anyway with garbage data) would either crash or silently produce a bogus run
// instead of the documented -1/error outcome.

TEST(SimulationManager, SimulationLoadErrorSkipsFactoryAndScoresNegativeOne) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{SimulationConfigData{}, {MissionConfigData{}}}};
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sim.yaml", ErrorRef{"SIM_CONFIG_INVALID", "bad simulation yaml"}},
         {ReferencedConfigFile{"mission.yaml"}}}};
    file_paths.drone_paths = {"drone.yaml"};
    file_paths.lidar_paths = {"lidar.yaml"};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(0);

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(
        composition, "tests/component/test_output/simulation_manager_test/sim_load_error", file_paths);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_EQ(report.runs[0].mission_results.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].status, MissionRunStatus::Error);
    ASSERT_EQ(report.runs[0].mission_results[0].errors.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "SIM_CONFIG_INVALID");
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].message, "bad simulation yaml");
}

TEST(SimulationManager, MissionLoadErrorSkipsFactoryAndScoresNegativeOne) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{SimulationConfigData{}, {MissionConfigData{}}}};
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sim.yaml"},
         {ReferencedConfigFile{"mission.yaml", ErrorRef{"MISSION_CONFIG_INVALID", "bad mission yaml"}}}}};
    file_paths.drone_paths = {"drone.yaml"};
    file_paths.lidar_paths = {"lidar.yaml"};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(0);

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(
        composition, "tests/component/test_output/simulation_manager_test/mission_load_error", file_paths);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_EQ(report.runs[0].mission_results.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].status, MissionRunStatus::Error);
    ASSERT_EQ(report.runs[0].mission_results[0].errors.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "MISSION_CONFIG_INVALID");
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].message, "bad mission yaml");
}

TEST(SimulationManager, LoadErrorRunStillReportsOutputResolutionFromSimulationConfig) {
    // Even a -1 placeholder run must report the resolution the hidden map was configured with
    // (mirrors SimulationRunFactoryImpl::outputMapResolution()'s formula:
    // simulation_config.map_resolution), not a meaningless default-zero -- SimulationOutputWriter
    // reads this field off whichever run it sees first for a mission, including an error run.
    // outputMapResolution() no longer consults the mission's requested factor at all (only
    // simulation_config.map_resolution is ever a value MapsComparison can score against), so this
    // must report simulation_config.map_resolution even though the group failed to load for an
    // unrelated reason.
    SimulationConfigData simulation;
    simulation.map_resolution = 20.0 * isq::length[cm];
    MissionConfigData mission;
    mission.gps_resolution = 10.0 * isq::length[cm];
    mission.output_mapping_resolution_factor = 2.0; // irrelevant to outputMapResolution() now
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{simulation, {mission}}};
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    CompositionFilePaths file_paths;
    file_paths.simulation_mission_paths = {
        {ReferencedConfigFile{"sim.yaml", ErrorRef{"SIM_CONFIG_INVALID", "bad"}}, {ReferencedConfigFile{"mission.yaml"}}}};
    file_paths.drone_paths = {"drone.yaml"};
    file_paths.lidar_paths = {"lidar.yaml"};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(0);

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report = manager.run(
        composition, "tests/component/test_output/simulation_manager_test/load_error_resolution", file_paths);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].output_map_config.resolution, 20.0 * isq::length[cm])
        << "an error placeholder run must still report simulation_config.map_resolution, not a "
           "default-zero value";
}

// ── run_factory_->create() / run->run() exception handling ──────────────────
//
// Per the assignment's Error Handling Policy, a failure in one run must score -1 and let the
// composition continue, not abort the remaining runs. SimulationException carries a stable
// machine-readable code; any other std::exception must fall back to a generic code rather than
// crash or leave the code empty.

TEST(SimulationManager, ExceptionFromFactoryCreateIsCaughtScoredNegativeOneAndDoesNotAbortRemainingRuns) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{SimulationConfigData{}, {MissionConfigData{}}}};
    composition.drone_configs = {DroneConfigData{}, DroneConfigData{}}; // 2 runs total
    composition.lidar_configs = {LidarConfigData{}};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    {
        ::testing::InSequence seq;
        EXPECT_CALL(*factory, create(_, _, _, _, _))
            .WillOnce(Throw(SimulationException("MAP_LOAD_FAILED", "could not load hidden map")));
        EXPECT_CALL(*factory, create(_, _, _, _, _))
            .WillOnce(Invoke([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                               const LidarConfigData&, const std::filesystem::path&) {
                auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
                ON_CALL(*run, run()).WillByDefault(Return(SimulationResult{}));
                return std::unique_ptr<ISimulationRun>(std::move(run));
            }));
    }

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report =
        manager.run(composition, "tests/component/test_output/simulation_manager_test/factory_throws");

    ASSERT_EQ(report.runs.size(), 2u)
        << "a thrown exception on one run must not abort the remaining runs in the composition";
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_EQ(report.runs[0].mission_results.size(), 1u);
    ASSERT_EQ(report.runs[0].mission_results[0].errors.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "MAP_LOAD_FAILED");
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].message, "could not load hidden map");
    EXPECT_EQ(report.runs[1].mission_score, 0.0) << "the second run should have completed normally";
}

TEST(SimulationManager, ExceptionFromRunRunUsesGenericErrorCodeForNonSimulationException) {
    SimulationCompositionData composition;
    composition.simulation_mission_groups = {{SimulationConfigData{}, {MissionConfigData{}}}};
    composition.drone_configs = {DroneConfigData{}};
    composition.lidar_configs = {LidarConfigData{}};

    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _))
        .WillByDefault(Invoke([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                                const LidarConfigData&, const std::filesystem::path&) {
            auto run = std::make_unique<NiceMock<test::GMockISimulationRun>>();
            EXPECT_CALL(*run, run()).WillOnce(Throw(std::runtime_error("disk full")));
            return std::unique_ptr<ISimulationRun>(std::move(run));
        }));

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report =
        manager.run(composition, "tests/component/test_output/simulation_manager_test/run_throws_generic");

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_EQ(report.runs[0].mission_results.size(), 1u);
    ASSERT_EQ(report.runs[0].mission_results[0].errors.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "RUN_INITIALIZATION_ERROR")
        << "a generic std::exception (not SimulationException) thrown from run->run() must fall "
           "back to a generic error code, not crash or leave the code empty";
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].message, "disk full");
}

// ── boundary: empty composition ──────────────────────────────────────────────

TEST(SimulationManager, EmptyCompositionProducesNoRunsAndDoesNotCrash) {
    const SimulationCompositionData composition; // no simulation_mission_groups, no drones, no lidars
    auto factory = std::make_unique<NiceMock<test::GMockISimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(0);

    SimulationManager manager(std::move(factory));
    const SimulationManagerReport report =
        manager.run(composition, "tests/component/test_output/simulation_manager_test/empty_composition");

    EXPECT_TRUE(report.runs.empty());
}
