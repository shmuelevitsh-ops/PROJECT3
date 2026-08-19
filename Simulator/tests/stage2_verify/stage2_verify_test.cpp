// Permanent coverage for Stage 2 of SIMULATOR_CORE_PLAN.md, merged into
// simulator_registration_test -- exercises simulator::parseCliOptions
// against the scenarios enumerated in the plan's Stage 2 "Verify" section,
// using real files (the already-built plugin .so's, inputs/sim_compose.yaml)
// and scratch directories for the folder-scanning cases.

#include <Simulator/CliOptions.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

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

const std::filesystem::path kSimulationFile = SIM_COMPOSE_PATH;
const std::filesystem::path kAlgorithmFile = ALGORITHM_PLUGIN_PATH;
const std::filesystem::path kMissionControlFile = MISSION_CONTROL_PLUGIN_PATH;

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

std::string simArg() { return "simulation=" + kSimulationFile.string(); }
std::string algoArg() { return "algorithm=" + kAlgorithmFile.string(); }
std::string mcArg() { return "mission_control=" + kMissionControlFile.string(); }

} // namespace

// ---- Comparative mode ----

TEST(Stage2Verify, ComparativeNoArgsRejected) {
    EXPECT_EQ(runParse({}), std::nullopt);
}

TEST(Stage2Verify, ComparativeMissingSimulationRejected) {
    ScratchDir mc_dir("stage2_verify_mc_missing_sim");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", "mission_control_folder=" + mc_dir.path().string(), algoArg()}), std::nullopt);
}

TEST(Stage2Verify, ComparativeMissingMissionControlFolderRejected) {
    EXPECT_EQ(runParse({"-comparative", simArg(), algoArg()}), std::nullopt);
}

TEST(Stage2Verify, ComparativeMissingAlgorithmRejected) {
    ScratchDir mc_dir("stage2_verify_mc_missing_algo");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string()}), std::nullopt);
}

TEST(Stage2Verify, ComparativeNonexistentSimulationFileRejected) {
    ScratchDir mc_dir("stage2_verify_mc_nonexistent_sim");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", "simulation=/nonexistent/path.yaml",
                        "mission_control_folder=" + mc_dir.path().string(), algoArg()}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeEmptyMissionControlFolderRejected) {
    ScratchDir mc_dir("stage2_verify_mc_empty");
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg()}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeOneRealSoInMissionControlFolderAccepted) {
    ScratchDir mc_dir("stage2_verify_mc_one_real");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / kMissionControlFile.filename());

    const auto parsed = runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg()});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::ComparativeOptions>(*parsed);
    EXPECT_EQ(options.mission_control_libraries.size(), 1u);
}

TEST(Stage2Verify, ComparativeMissionControlFolderIgnoresDanglingSymlink) {
    ScratchDir mc_dir("stage2_verify_mc_dangling_symlink");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / kMissionControlFile.filename());
    std::filesystem::create_symlink("/nonexistent/target", mc_dir.path() / "broken_link.so");

    const auto parsed =
        runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg()});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::ComparativeOptions>(*parsed);
    EXPECT_EQ(options.mission_control_libraries.size(), 1u);
}

TEST(Stage2Verify, ComparativeUnsupportedKeyRejected) {
    ScratchDir mc_dir("stage2_verify_mc_unsupported");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(),
                        "foo=bar"}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeModeFlagLastIsAccepted) {
    ScratchDir mc_dir("stage2_verify_mc_flag_last");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / kMissionControlFile.filename());

    const auto parsed =
        runParse({simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(), "-comparative"});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::ComparativeOptions>(*parsed);
    EXPECT_EQ(options.simulation_composition_file, kSimulationFile);
    EXPECT_EQ(options.mission_control_folder, mc_dir.path());
    EXPECT_EQ(options.algorithm_so_file, kAlgorithmFile);
    EXPECT_EQ(options.mission_control_libraries.size(), 1u);
    EXPECT_FALSE(options.num_threads.has_value());
    EXPECT_FALSE(options.verbose);
}

TEST(Stage2Verify, ComparativeNumThreads2Accepted) {
    ScratchDir mc_dir("stage2_verify_mc_nt2");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / kMissionControlFile.filename());

    const auto parsed = runParse(
        {"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(), "num_threads=2"});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::ComparativeOptions>(*parsed);
    ASSERT_TRUE(options.num_threads.has_value());
    EXPECT_EQ(*options.num_threads, 2);
}

TEST(Stage2Verify, ComparativeNumThreadsZeroRejected) {
    ScratchDir mc_dir("stage2_verify_mc_nt0");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(),
                        "num_threads=0"}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeNumThreadsNegativeRejected) {
    ScratchDir mc_dir("stage2_verify_mc_nt_neg");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(),
                        "num_threads=-1"}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeNumThreadsNonNumericRejected) {
    ScratchDir mc_dir("stage2_verify_mc_nt_abc");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg(),
                        "num_threads=abc"}),
              std::nullopt);
}

TEST(Stage2Verify, ComparativeRepeatedKeyRejected) {
    ScratchDir mc_dir("stage2_verify_mc_repeated");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", simArg(), simArg(), "mission_control_folder=" + mc_dir.path().string(),
                        algoArg()}),
              std::nullopt);
}

TEST(Stage2Verify, BothModeFlagsRejected) {
    ScratchDir mc_dir("stage2_verify_both_modes");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-comparative", "-competition", simArg(), "mission_control_folder=" + mc_dir.path().string(),
                        algoArg()}),
              std::nullopt);
}

// ---- Competition mode ----

TEST(Stage2Verify, CompetitionValidInvocationAccepted) {
    ScratchDir algo_dir("stage2_verify_algo_valid");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());

    const auto parsed =
        runParse({"-competition", simArg(), mcArg(), "algorithms_folder=" + algo_dir.path().string()});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::CompetitionOptions>(*parsed);
    EXPECT_EQ(options.simulation_composition_file, kSimulationFile);
    EXPECT_EQ(options.mission_control_so_file, kMissionControlFile);
    EXPECT_EQ(options.algorithms_folder, algo_dir.path());
    EXPECT_EQ(options.algorithm_libraries.size(), 1u);
}

TEST(Stage2Verify, CompetitionMissingSimulationRejected) {
    ScratchDir algo_dir("stage2_verify_algo_missing_sim");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());
    EXPECT_EQ(runParse({"-competition", mcArg(), "algorithms_folder=" + algo_dir.path().string()}), std::nullopt);
}

TEST(Stage2Verify, CompetitionMissingMissionControlRejected) {
    ScratchDir algo_dir("stage2_verify_algo_missing_mc");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());
    EXPECT_EQ(runParse({"-competition", simArg(), "algorithms_folder=" + algo_dir.path().string()}), std::nullopt);
}

TEST(Stage2Verify, CompetitionMissingAlgorithmsFolderRejected) {
    EXPECT_EQ(runParse({"-competition", simArg(), mcArg()}), std::nullopt);
}

TEST(Stage2Verify, CompetitionEmptyAlgorithmsFolderRejected) {
    ScratchDir algo_dir("stage2_verify_algo_empty");
    EXPECT_EQ(runParse({"-competition", simArg(), mcArg(), "algorithms_folder=" + algo_dir.path().string()}),
              std::nullopt);
}

TEST(Stage2Verify, CompetitionOneRealSoInAlgorithmsFolderAccepted) {
    ScratchDir algo_dir("stage2_verify_algo_one_real");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());

    const auto parsed =
        runParse({"-competition", simArg(), mcArg(), "algorithms_folder=" + algo_dir.path().string()});
    ASSERT_TRUE(parsed.has_value());
    const auto& options = std::get<simulator::CompetitionOptions>(*parsed);
    EXPECT_EQ(options.algorithm_libraries.size(), 1u);
}

TEST(Stage2Verify, CompetitionRepeatedKeyRejected) {
    ScratchDir algo_dir("stage2_verify_algo_repeated");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / kAlgorithmFile.filename());
    EXPECT_EQ(runParse({"-competition", simArg(), mcArg(), mcArg(), "algorithms_folder=" + algo_dir.path().string()}),
              std::nullopt);
}

TEST(Stage2Verify, CompetitionComparativeOnlyKeysRejectedAsUnsupported) {
    ScratchDir mc_dir("stage2_verify_competition_wrong_keys");
    std::ofstream(mc_dir.path() / "a.so").put('x');
    EXPECT_EQ(runParse({"-competition", simArg(), "mission_control_folder=" + mc_dir.path().string(), algoArg()}),
              std::nullopt);
}
