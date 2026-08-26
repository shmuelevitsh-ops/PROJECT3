// Permanent coverage for Stage 3 of SIMULATOR_CORE_PLAN.md, merged into
// simulator_registration_test -- exercises Simulator::runComparative /
// Simulator::runCompetition directly against the scenarios enumerated in the
// plan's Stage 3 "Verify" section: multi-component runs, a garbage .so landing
// in the errors: list without stopping the rest, deterministic alphabetical
// processing order, a fixed-component load failure propagating uncaught, and
// a run/write-phase failure isolating only the one affected component.
//
// Uses a synthetic one-simulation/one-mission/one-drone/one-lidar composition
// (small_simulation_out/small_mission_out/drone_small/lidar_short -- the same
// fast, overlapping-bounds combo established in Stage 1's own verification)
// rather than the full inputs/sim_compose.yaml, which takes several minutes
// per component and previously crashed the devcontainer.

#include <Simulator/CerrContextGuard.h>
#include <Simulator/CliOptions.h>
#include <Simulator/Simulator.h>

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace {

const std::filesystem::path kAlgorithmFile = ALGORITHM_PLUGIN_PATH;
const std::filesystem::path kMissionControlFile = MISSION_CONTROL_PLUGIN_PATH;
const std::filesystem::path kInputsDir = TEST_INPUTS_DIR;

class ScratchDir {
public:
    explicit ScratchDir(const std::string& name) : path_(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ~ScratchDir() { std::filesystem::remove_all(path_); }

    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

// Redirects std::cerr into an in-memory buffer for its lifetime, restoring the
// original target on destruction -- lets a test inspect error.log-equivalent
// output (including CerrContextGuard's nested prefixes) without a real file.
class CerrCapture {
public:
    CerrCapture() : sink_(buffer_.rdbuf()) {}

    [[nodiscard]] std::string str() const { return buffer_.str(); }

private:
    std::ostringstream buffer_;
    simulator::CerrSinkGuard sink_; // must be constructed after buffer_ (member init order below)
};

std::optional<std::variant<simulator::ComparativeOptions, simulator::CompetitionOptions>>
runParse(const std::vector<std::string>& args) {
    std::vector<std::string> owned = args;
    std::vector<char*> argv;
    static std::string prog = "simulator_322889890_315113738";
    argv.push_back(prog.data());
    for (std::string& arg : owned) {
        argv.push_back(arg.data());
    }
    return simulator::parseCliOptions(static_cast<int>(argv.size()), argv.data());
}

simulator::ComparativeOptions parseComparative(const std::vector<std::string>& args) {
    return std::get<simulator::ComparativeOptions>(*runParse(args));
}

simulator::CompetitionOptions parseCompetition(const std::vector<std::string>& args) {
    return std::get<simulator::CompetitionOptions>(*runParse(args));
}

// A minimal one-simulation/one-mission/one-drone/one-lidar composition file,
// written into `dir`, referencing the real (fast) small_simulation_out fixture
// files under inputs/ by absolute path.
std::filesystem::path writeMinimalComposition(const std::filesystem::path& dir) {
    const std::filesystem::path compose_path = dir / "mini_compose.yaml";
    std::ofstream out(compose_path);
    out << "simulation_compositions:\n"
           "  simulations:\n"
           "    - simulation_config: \""
        << (kInputsDir / "simulation/small_simulation_out.yaml").string()
        << "\"\n"
           "      mission_configs:\n"
           "        - \""
        << (kInputsDir / "mission/small_mission_out.yaml").string()
        << "\"\n"
           "  drone_configs:\n"
           "    - \""
        << (kInputsDir / "drone/drone_small.yaml").string()
        << "\"\n"
           "  lidar_configs:\n"
           "    - \""
        << (kInputsDir / "lidar/lidar_short.yaml").string()
        << "\"\n";
    return compose_path;
}

void writeGarbageSo(const std::filesystem::path& path) {
    std::ofstream garbage(path);
    garbage << "not a real shared library";
}

} // namespace

TEST(Stage3Verify, ComparativeMultiComponentGarbageNestingAndOrdering) {
    ScratchDir mc_dir("stage3_verify_mc_multi");
    // Created in non-alphabetical order on purpose, so a passing ordering
    // assertion below can only be explained by an actual sort, not by the
    // directory scan happening to already be alphabetical.
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "zzz_mgr.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "aaa_mgr.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mmm_mgr.so");
    writeGarbageSo(mc_dir.path() / "garbage.so");

    ScratchDir compose_dir("stage3_verify_compose_multi");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    // A dedicated copy of the algorithm .so: Registrar loads each unique library *path* at most
    // once per process (a second dlopen of an already-mapped path does not re-run its REGISTER_*
    // constructor, so Registrar reports "did not register..." for the redundant load) -- since
    // this whole binary is one process across every TEST(), each test that needs a real,
    // successfully-loadable fixed component must use a path no earlier test already loaded.
    ScratchDir algo_dir("stage3_verify_algo_multi");
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(), "algorithm=" + algorithm_copy.string()});

    ScratchDir results_dir("stage3_verify_results_multi");

    CerrCapture capture;
    const std::size_t ran = simulator::Simulator().runComparative(options, results_dir.path());
    const std::string log = capture.str();

    EXPECT_EQ(ran, 3u);

    for (const std::string stem : {"aaa_mgr", "mmm_mgr", "zzz_mgr"}) {
        EXPECT_TRUE(std::filesystem::exists(results_dir.path() / ("simulation_output_" + stem + ".yaml")));
        EXPECT_TRUE(std::filesystem::exists(results_dir.path() / stem / "simulations"));
    }
    EXPECT_FALSE(std::filesystem::exists(results_dir.path() / "simulation_output_garbage.yaml"));

    EXPECT_NE(log.find("failed to load garbage.so:"), std::string::npos);
    // Outer component-level prefix nesting with SimulationManager's inner per-run prefix.
    EXPECT_NE(log.find("[component=aaa_mgr.so] [sim="), std::string::npos);

    const YAML::Node root = YAML::LoadFile((results_dir.path() / "comparative_report.yaml").string());
    const YAML::Node comparative = root["comparative_report"];

    std::vector<std::string> errors;
    for (const YAML::Node& entry : comparative["errors"]) {
        errors.push_back(entry.as<std::string>());
    }
    EXPECT_NE(std::find(errors.begin(), errors.end(), "garbage.so"), errors.end());

    // The three components are byte-identical .so copies run against the same composition, so
    // they score identically and land in one same_results group -- whose member order is exactly
    // the loop's processing order (computeComponentTotals is appended in loop order, and grouping
    // preserves first-occurrence order within a group). Their on-disk creation order above was
    // deliberately non-alphabetical (zzz, aaa, mmm), so this order can only be explained by an
    // actual sort -- a single run is already conclusive proof, without needing to repeat it.
    ASSERT_EQ(comparative["results_summary"].size(), 1u);
    std::vector<std::string> order;
    for (const YAML::Node& name : comparative["results_summary"][0]["same_results"]) {
        order.push_back(name.as<std::string>());
    }
    EXPECT_EQ(order, (std::vector<std::string>{"aaa_mgr.so", "mmm_mgr.so", "zzz_mgr.so"}));
}

TEST(Stage3Verify, ComparativeFixedComponentLoadFailurePropagates) {
    ScratchDir mc_dir("stage3_verify_mc_fixed_fail");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / kMissionControlFile.filename());

    ScratchDir bogus_dir("stage3_verify_bogus_algo");
    const std::filesystem::path bogus_algo = bogus_dir.path() / "not_a_library.so";
    writeGarbageSo(bogus_algo);

    ScratchDir compose_dir("stage3_verify_compose_fixed_fail");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("stage3_verify_results_fixed_fail");

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(), "algorithm=" + bogus_algo.string()});

    EXPECT_THROW((void)simulator::Simulator().runComparative(options, results_dir.path()), std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(results_dir.path()));
}

TEST(Stage3Verify, CompetitionFixedComponentLoadFailurePropagates) {
    ScratchDir algo_dir("stage3_verify_algo_fixed_fail");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());

    ScratchDir bogus_dir("stage3_verify_bogus_mc");
    const std::filesystem::path bogus_mc = bogus_dir.path() / "not_a_library.so";
    writeGarbageSo(bogus_mc);

    ScratchDir compose_dir("stage3_verify_compose_fixed_fail_competition");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("stage3_verify_results_fixed_fail_competition");

    const simulator::CompetitionOptions options =
        parseCompetition({"-competition", "simulation=" + compose_path.string(), "mission_control=" + bogus_mc.string(),
                          "algorithms_folder=" + algo_dir.path().string()});

    EXPECT_THROW((void)simulator::Simulator().runCompetition(options, results_dir.path()), std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(results_dir.path()));
}

TEST(Stage3Verify, ComparativeRunWritePhaseFailureIsolatesOneComponent) {
    ScratchDir mc_dir("stage3_verify_mc_blocked");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_ok.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_blocked.so");

    // A dedicated copy: see the comment in ComparativeMultiComponentGarbageNestingAndOrdering on
    // why the fixed algorithm .so can't be the same already-loaded path an earlier test used.
    ScratchDir algo_dir("stage3_verify_algo_blocked");
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    ScratchDir compose_dir("stage3_verify_compose_blocked");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("stage3_verify_results_blocked");
    // Pre-create the blocked component's own map-output directory *as a regular file*, so
    // SimulationManager's internal create_directories(leaf_dir) fails for that component only.
    { std::ofstream blocker(results_dir.path() / "mgr_blocked"); }

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(), "algorithm=" + algorithm_copy.string()});

    CerrCapture capture;
    const std::size_t ran = simulator::Simulator().runComparative(options, results_dir.path());
    const std::string log = capture.str();

    EXPECT_EQ(ran, 1u);
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_ok.yaml"));
    EXPECT_FALSE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_blocked.yaml"));
    EXPECT_NE(log.find("component mgr_blocked.so failed during simulation/output:"), std::string::npos);

    const YAML::Node root = YAML::LoadFile((results_dir.path() / "comparative_report.yaml").string());
    const YAML::Node comparative = root["comparative_report"];

    std::vector<std::string> errors;
    for (const YAML::Node& entry : comparative["errors"]) {
        errors.push_back(entry.as<std::string>());
    }
    EXPECT_NE(std::find(errors.begin(), errors.end(), "mgr_blocked.so"), errors.end());
    EXPECT_EQ(std::find(errors.begin(), errors.end(), "mgr_ok.so"), errors.end());

    std::vector<std::string> summarized;
    for (const YAML::Node& group : comparative["results_summary"]) {
        for (const YAML::Node& name : group["same_results"]) {
            summarized.push_back(name.as<std::string>());
        }
    }
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "mgr_ok.so"), summarized.end());
    EXPECT_EQ(std::find(summarized.begin(), summarized.end(), "mgr_blocked.so"), summarized.end());
}

TEST(Stage3Verify, CompetitionMultiComponentAndGarbage) {
    ScratchDir algo_dir("stage3_verify_algo_multi");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / "bbb_algo.so");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / "aaa_algo.so");
    writeGarbageSo(algo_dir.path() / "garbage.so");

    ScratchDir compose_dir("stage3_verify_compose_competition_multi");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("stage3_verify_results_competition_multi");

    // A dedicated copy of the fixed mission-control .so: see the comment in
    // ComparativeMultiComponentGarbageNestingAndOrdering on why the fixed component can't reuse
    // the raw kMissionControlFile path once other tests in this same process may have loaded it.
    ScratchDir mc_dir("stage3_verify_mc_competition_multi_fixed");
    const std::filesystem::path mission_control_copy = mc_dir.path() / kMissionControlFile.filename();
    std::filesystem::copy_file(kMissionControlFile, mission_control_copy);

    const simulator::CompetitionOptions options =
        parseCompetition({"-competition", "simulation=" + compose_path.string(),
                          "mission_control=" + mission_control_copy.string(),
                          "algorithms_folder=" + algo_dir.path().string()});

    CerrCapture capture;
    const std::size_t ran = simulator::Simulator().runCompetition(options, results_dir.path());
    const std::string log = capture.str();

    EXPECT_EQ(ran, 2u);
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_aaa_algo.yaml"));
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_bbb_algo.yaml"));
    EXPECT_FALSE(std::filesystem::exists(results_dir.path() / "simulation_output_garbage.yaml"));
    EXPECT_NE(log.find("failed to load garbage.so:"), std::string::npos);

    const YAML::Node root = YAML::LoadFile((results_dir.path() / "competitive_report.yaml").string());
    const YAML::Node competitive = root["competitive_report"];

    std::vector<std::string> errors;
    for (const YAML::Node& entry : competitive["errors"]) {
        errors.push_back(entry.as<std::string>());
    }
    EXPECT_NE(std::find(errors.begin(), errors.end(), "garbage.so"), errors.end());

    std::vector<std::string> summarized;
    for (const YAML::Node& entry : competitive["results_summary"]) {
        summarized.push_back(entry["algorithm"].as<std::string>());
    }
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "aaa_algo.so"), summarized.end());
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "bbb_algo.so"), summarized.end());
}
