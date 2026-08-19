// Permanent coverage for Stage 2 of MULTI_THREADS_PLAN.md, merged into
// simulator_registration_test -- exercises simulator::computeWorkerCount directly, and
// simulator::runComparative/runCompetition under num_threads >= 2 against the scenarios
// enumerated in the plan's Stage 2 "Verify" section: concurrent execution reproduces the exact
// same deterministic results_summary/errors ordering as sequential execution (for both comparative
// and competitive mode), fewer components than requested threads doesn't crash or hang, a
// run/write-phase failure still isolates exactly one component under real concurrency, error.log
// stays complete/non-interleaved/correctly attributed under concurrency (never asserting on line
// order), and the returned "ran N component(s)" count is correct under a mix of successes and
// failures.
//
// Uses the same synthetic one-simulation/one-mission/one-drone/one-lidar composition established
// by Stage 1/3's own verification (small_simulation_out/small_mission_out/drone_small/lidar_short)
// for fast, real pipeline execution.
//
// Registrar loads each unique library *path* at most once per process (a second dlopen of an
// already-mapped path does not re-run its REGISTER_* constructor) -- since this whole binary is
// one process across every TEST(), every scratch .so path used below (looped components AND the
// one fixed component per call) is freshly std::filesystem::copy_file'd, including across repeated
// runComparative/runCompetition calls within the same test, matching the convention already
// established in stage3_verify_test.cpp.
//
// Every scenario below that runs a real, concurrent runComparative/runCompetition (i.e. every one
// except the dedicated error.log test, which installs its own CerrSinkGuard over a real file)
// installs a CerrCapture -- a CerrSinkGuard over an in-memory buffer -- around the call, even when
// the test itself doesn't inspect the captured text. Without one, a load failure logged from
// multiple worker threads writes straight to the test binary's raw, unsynchronized default
// std::cerr streambuf, whose multi-thread interleaving behavior the C++ standard leaves
// unspecified (see CerrContextGuard.h's CerrSinkGuard doc comment) -- installing the sink matches
// what main() always does in production (Stage 1) before calling runComparative/runCompetition,
// and keeps every test's console output clean of interleaved/garbled lines regardless of whether
// that test cares about the log's content.

#include <Simulator/CerrContextGuard.h>
#include <Simulator/CliOptions.h>
#include <Simulator/SimulatorRunner.h>

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <variant>
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

    ScratchDir(const ScratchDir&) = delete;
    ScratchDir& operator=(const ScratchDir&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
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

// Installs a CerrSinkGuard over an in-memory buffer for its lifetime -- lets a test run a real
// concurrent runComparative/runCompetition without raw, unsynchronized std::cerr writes from
// multiple worker threads interleaving onto the test binary's own stderr (see the file-level
// comment above). Most call sites below never inspect str(); the object exists purely to keep
// std::cerr's writes funneled through Stage 1's thread-safe sink for the scope's duration.
class CerrCapture {
public:
    CerrCapture() : sink_(buffer_.rdbuf()) {}

    [[nodiscard]] std::string str() const { return buffer_.str(); }

private:
    std::ostringstream buffer_;
    simulator::CerrSinkGuard sink_; // must be constructed after buffer_ (member init order below)
};

// A minimal one-simulation/one-mission/one-drone/one-lidar composition file, written into `dir`,
// referencing the real (fast) small_simulation_out fixture files under inputs/ by absolute path.
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

struct ResultsSummaryGroup {
    std::vector<std::string> same_results;
    double total_score = 0.0;
    std::size_t total_steps = 0;

    bool operator==(const ResultsSummaryGroup& other) const {
        return same_results == other.same_results && total_score == other.total_score &&
               total_steps == other.total_steps;
    }
};

std::vector<ResultsSummaryGroup> extractResultsSummary(const YAML::Node& report) {
    std::vector<ResultsSummaryGroup> groups;
    for (const YAML::Node& group : report["results_summary"]) {
        ResultsSummaryGroup entry;
        for (const YAML::Node& name : group["same_results"]) {
            entry.same_results.push_back(name.as<std::string>());
        }
        entry.total_score = group["total_score"].as<double>();
        entry.total_steps = group["total_steps"].as<std::size_t>();
        groups.push_back(std::move(entry));
    }
    return groups;
}

std::vector<std::string> extractErrors(const YAML::Node& report) {
    std::vector<std::string> errors;
    for (const YAML::Node& entry : report["errors"]) {
        errors.push_back(entry.as<std::string>());
    }
    return errors;
}

struct CompetitiveResultEntry {
    std::string algorithm;
    double total_score = 0.0;
    std::size_t total_steps = 0;

    bool operator==(const CompetitiveResultEntry& other) const {
        return algorithm == other.algorithm && total_score == other.total_score &&
               total_steps == other.total_steps;
    }
};

std::vector<CompetitiveResultEntry> extractCompetitiveResultsSummary(const YAML::Node& report) {
    std::vector<CompetitiveResultEntry> entries;
    for (const YAML::Node& entry : report["results_summary"]) {
        CompetitiveResultEntry item;
        item.algorithm = entry["algorithm"].as<std::string>();
        item.total_score = entry["total_score"].as<double>();
        item.total_steps = entry["total_steps"].as<std::size_t>();
        entries.push_back(std::move(item));
    }
    return entries;
}

// Builds one complete, self-contained scratch setup (its own mission_control_folder with
// `mc_filenames` copies plus its own fixed algorithm .so copy, per the Registrar one-load-per-path
// constraint above) and runs simulator::runComparative against it, writing results into
// `results_dir` (owned by the caller, so it survives after this function's own scratch
// directories -- which are no longer needed once runComparative has returned -- are cleaned up).
std::size_t runComparativeScratch(const std::string& tag, const std::optional<int>& num_threads,
                                  const std::vector<std::string>& mc_filenames,
                                  const std::filesystem::path& results_dir) {
    ScratchDir mc_dir("mt_verify_mc_" + tag);
    for (const std::string& name : mc_filenames) {
        std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / name);
    }
    ScratchDir compose_dir("mt_verify_compose_" + tag);
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());
    ScratchDir algo_dir("mt_verify_algo_" + tag);
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    std::vector<std::string> args = {"-comparative", "simulation=" + compose_path.string(),
                                     "mission_control_folder=" + mc_dir.path().string(),
                                     "algorithm=" + algorithm_copy.string()};
    if (num_threads.has_value()) {
        args.push_back("num_threads=" + std::to_string(*num_threads));
    }
    const simulator::ComparativeOptions options = parseComparative(args);

    CerrCapture capture;
    return simulator::runComparative(options, results_dir);
}

// Symmetric to runComparativeScratch, but for runCompetition: builds one complete, self-contained
// scratch algorithms_folder (three healthy, out-of-alphabetical-order Algorithm_*.so copies plus
// one garbage.so) and its own fixed mission_control .so copy, runs it under num_threads=4, and
// captures/discards std::cerr through CerrCapture for the same reason runComparativeScratch does.
std::size_t runCompetitionScratchMixed(const std::string& tag, const std::filesystem::path& results_dir) {
    ScratchDir algo_dir("mt_verify_algo_competition_" + tag);
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / "zzz_algo.so");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / "aaa_algo.so");
    std::filesystem::copy_file(kAlgorithmFile, algo_dir.path() / "mmm_algo.so");
    writeGarbageSo(algo_dir.path() / "garbage_algo.so");

    ScratchDir compose_dir("mt_verify_compose_competition_" + tag);
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir mc_dir("mt_verify_mc_competition_" + tag);
    const std::filesystem::path mission_control_copy = mc_dir.path() / kMissionControlFile.filename();
    std::filesystem::copy_file(kMissionControlFile, mission_control_copy);

    const simulator::CompetitionOptions options =
        parseCompetition({"-competition", "simulation=" + compose_path.string(),
                          "mission_control=" + mission_control_copy.string(),
                          "algorithms_folder=" + algo_dir.path().string(), "num_threads=4"});

    CerrCapture capture;
    return simulator::runCompetition(options, results_dir);
}

} // namespace

TEST(SimulatorRunnerVerify, ComputeWorkerCount) {
    EXPECT_EQ(simulator::computeWorkerCount(std::nullopt, 5), 0u);
    EXPECT_EQ(simulator::computeWorkerCount(1, 5), 0u);
    EXPECT_EQ(simulator::computeWorkerCount(5, 1), 0u); // the critical never-total-2 case
    EXPECT_EQ(simulator::computeWorkerCount(4, 1), 0u);
    EXPECT_EQ(simulator::computeWorkerCount(4, 10), 4u);
    EXPECT_EQ(simulator::computeWorkerCount(20, 10), 10u);
}

TEST(MultithreadingVerify, ConcurrentMultiComponentMatchesSequentialDeterministicOrder) {
    // Deliberately out-of-alphabetical-order filenames, same technique
    // Stage3Verify.ComparativeMultiComponentGarbageNestingAndOrdering already uses.
    const std::vector<std::string> mc_filenames = {"zzz_mgr.so", "aaa_mgr.so", "mmm_mgr.so",
                                                    "ccc_mgr.so", "bbb_mgr.so", "yyy_mgr.so"};

    ScratchDir sequential_results("mt_verify_results_sequential");
    const std::size_t sequential_ran =
        runComparativeScratch("sequential", std::nullopt, mc_filenames, sequential_results.path());
    const YAML::Node sequential_report =
        YAML::LoadFile((sequential_results.path() / "comparative_report.yaml").string())["comparative_report"];
    const std::vector<ResultsSummaryGroup> sequential_summary = extractResultsSummary(sequential_report);
    const std::vector<std::string> sequential_errors = extractErrors(sequential_report);

    EXPECT_EQ(sequential_ran, mc_filenames.size());
    for (const std::string& name : mc_filenames) {
        const std::string stem = std::filesystem::path(name).stem().string();
        EXPECT_TRUE(std::filesystem::exists(sequential_results.path() / ("simulation_output_" + stem + ".yaml")));
    }

    for (int repeat = 0; repeat < 3; ++repeat) {
        ScratchDir concurrent_results("mt_verify_results_concurrent_" + std::to_string(repeat));
        const std::size_t concurrent_ran = runComparativeScratch(
            "concurrent_" + std::to_string(repeat), 4, mc_filenames, concurrent_results.path());
        const YAML::Node concurrent_report =
            YAML::LoadFile((concurrent_results.path() / "comparative_report.yaml").string())["comparative_report"];

        EXPECT_EQ(concurrent_ran, sequential_ran);
        EXPECT_EQ(extractResultsSummary(concurrent_report), sequential_summary);
        EXPECT_EQ(extractErrors(concurrent_report), sequential_errors);
        for (const std::string& name : mc_filenames) {
            const std::string stem = std::filesystem::path(name).stem().string();
            EXPECT_TRUE(
                std::filesystem::exists(concurrent_results.path() / ("simulation_output_" + stem + ".yaml")));
        }
    }
}

TEST(MultithreadingVerify, FewerComponentsThanRequestedThreadsDoesNotCrashOrHang) {
    const std::vector<std::string> mc_filenames = {"only_a.so", "only_b.so"};
    ScratchDir results_dir("mt_verify_results_fewer_than_threads");

    const std::size_t ran = runComparativeScratch("fewer_than_threads", 8, mc_filenames, results_dir.path());

    EXPECT_EQ(ran, 2u);
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_only_a.yaml"));
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_only_b.yaml"));
}

TEST(MultithreadingVerify, RunWritePhaseFailureIsolatesOneComponentUnderConcurrency) {
    ScratchDir mc_dir("mt_verify_mc_blocked");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_ok_1.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_ok_2.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_ok_3.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "mgr_blocked.so");

    ScratchDir algo_dir("mt_verify_algo_blocked");
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    ScratchDir compose_dir("mt_verify_compose_blocked");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("mt_verify_results_blocked");
    // Pre-create the blocked component's own map-output directory *as a regular file*, so
    // SimulationManager's internal create_directories(leaf_dir) fails for that component only.
    { std::ofstream blocker(results_dir.path() / "mgr_blocked"); }

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(),
                          "algorithm=" + algorithm_copy.string(), "num_threads=4"});

    CerrCapture capture;
    const std::size_t ran = simulator::runComparative(options, results_dir.path());
    const std::string log = capture.str();

    EXPECT_EQ(ran, 3u);
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_ok_1.yaml"));
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_ok_2.yaml"));
    EXPECT_TRUE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_ok_3.yaml"));
    EXPECT_FALSE(std::filesystem::exists(results_dir.path() / "simulation_output_mgr_blocked.yaml"));
    EXPECT_NE(log.find("component mgr_blocked.so failed during simulation/output:"), std::string::npos);

    const YAML::Node report =
        YAML::LoadFile((results_dir.path() / "comparative_report.yaml").string())["comparative_report"];

    const std::vector<std::string> errors = extractErrors(report);
    EXPECT_NE(std::find(errors.begin(), errors.end(), "mgr_blocked.so"), errors.end());
    EXPECT_EQ(errors.size(), 1u);

    std::vector<std::string> summarized;
    for (const ResultsSummaryGroup& group : extractResultsSummary(report)) {
        for (const std::string& name : group.same_results) {
            summarized.push_back(name);
        }
    }
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "mgr_ok_1.so"), summarized.end());
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "mgr_ok_2.so"), summarized.end());
    EXPECT_NE(std::find(summarized.begin(), summarized.end(), "mgr_ok_3.so"), summarized.end());
    EXPECT_EQ(std::find(summarized.begin(), summarized.end(), "mgr_blocked.so"), summarized.end());
}

namespace {

// Runs one error.log-under-concurrency scenario against a fresh scratch setup (own looped .so
// copies, including one garbage.so, plus its own fixed algorithm .so copy), capturing error.log
// through a real on-disk file (so CerrSinkGuard wraps a real std::ofstream's buffer, not just an
// in-memory one). Both the std::ofstream and the CerrSinkGuard installed over it are fully
// destroyed (in an inner block) before the file is reopened for reading -- Stage 1 performs no
// per-line flush, so final on-disk content is only guaranteed once error_log's own destructor has
// actually run.
std::string runScenarioCapturingErrorLog(const std::string& tag, std::size_t& ran_out) {
    ScratchDir mc_dir("mt_verify_mc_errorlog_" + tag);
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "healthy_1.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "healthy_2.so");
    writeGarbageSo(mc_dir.path() / "garbage_1.so");
    writeGarbageSo(mc_dir.path() / "garbage_2.so");

    ScratchDir algo_dir("mt_verify_algo_errorlog_" + tag);
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    ScratchDir compose_dir("mt_verify_compose_errorlog_" + tag);
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("mt_verify_results_errorlog_" + tag);
    ScratchDir log_dir("mt_verify_log_errorlog_" + tag);
    const std::filesystem::path error_log_path = log_dir.path() / "error.log";

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(),
                          "algorithm=" + algorithm_copy.string(), "num_threads=4"});

    {
        std::ofstream error_log(error_log_path);
        const simulator::CerrSinkGuard sink(error_log.rdbuf());
        ran_out = simulator::runComparative(options, results_dir.path());
    } // sink destroyed (restores std::cerr), then error_log destroyed (flushes to disk).

    std::ifstream in(error_log_path);
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

} // namespace

TEST(MultithreadingVerify, ErrorLogUnderConcurrencyIsCompleteNonInterleavedAndAttributed) {
    for (int repeat = 0; repeat < 3; ++repeat) {
        std::size_t ran = 0;
        const std::string log = runScenarioCapturingErrorLog(std::to_string(repeat), ran);

        EXPECT_EQ(ran, 2u);

        std::istringstream lines(log);
        std::string line;
        std::vector<std::string> unattributed_lines;
        while (std::getline(lines, line)) {
            if (line.empty()) {
                continue;
            }
            const bool is_component_prefixed = line.rfind("[component=", 0) == 0;
            const bool is_known_top_level_message =
                line.find("parseMissionConfig: output_mapping_resolution_factor missing") != std::string::npos;
            if (!is_component_prefixed && !is_known_top_level_message) {
                unattributed_lines.push_back(line);
            }
        }
        EXPECT_TRUE(unattributed_lines.empty())
            << "found line(s) that don't match any expected shape (possible interleaving): "
            << (unattributed_lines.empty() ? "" : unattributed_lines.front());

        // Completeness + attribution: each garbage component's failure is logged exactly once,
        // under its own [component=...] prefix. Only the stable portion of the message (the
        // literal "failed to load " text) is asserted -- the exception text appended after it
        // embeds this repeat's own scratch path, which legitimately differs between repeats.
        for (const std::string& garbage_name : {std::string("garbage_1.so"), std::string("garbage_2.so")}) {
            const std::string expected_prefix = "[component=" + garbage_name + "] failed to load " + garbage_name + ":";
            std::size_t occurrences = 0;
            std::size_t pos = 0;
            while ((pos = log.find(expected_prefix, pos)) != std::string::npos) {
                ++occurrences;
                pos += expected_prefix.size();
            }
            EXPECT_EQ(occurrences, 1u) << "expected exactly one load-failure line for " << garbage_name;
        }

        // The two healthy components must not appear anywhere in an error/failure line.
        for (const std::string& healthy_name : {std::string("healthy_1.so"), std::string("healthy_2.so")}) {
            EXPECT_EQ(log.find("failed to load " + healthy_name), std::string::npos);
            EXPECT_EQ(log.find("component " + healthy_name + " failed"), std::string::npos);
        }
    }
}

TEST(MultithreadingVerify, ReturnValueCorrectWithMixOfSuccessesAndFailuresUnderConcurrency) {
    ScratchDir mc_dir("mt_verify_mc_mixed_return");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "ok_1.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "ok_2.so");
    std::filesystem::copy_file(kMissionControlFile, mc_dir.path() / "ok_3.so");
    writeGarbageSo(mc_dir.path() / "garbage_1.so");
    writeGarbageSo(mc_dir.path() / "garbage_2.so");

    ScratchDir algo_dir("mt_verify_algo_mixed_return");
    const std::filesystem::path algorithm_copy = algo_dir.path() / kAlgorithmFile.filename();
    std::filesystem::copy_file(kAlgorithmFile, algorithm_copy);

    ScratchDir compose_dir("mt_verify_compose_mixed_return");
    const std::filesystem::path compose_path = writeMinimalComposition(compose_dir.path());

    ScratchDir results_dir("mt_verify_results_mixed_return");

    const simulator::ComparativeOptions options =
        parseComparative({"-comparative", "simulation=" + compose_path.string(),
                          "mission_control_folder=" + mc_dir.path().string(),
                          "algorithm=" + algorithm_copy.string(), "num_threads=4"});

    CerrCapture capture;
    const std::size_t ran = simulator::runComparative(options, results_dir.path());

    EXPECT_EQ(ran, 3u);

    const YAML::Node report =
        YAML::LoadFile((results_dir.path() / "comparative_report.yaml").string())["comparative_report"];
    EXPECT_EQ(extractErrors(report).size(), 2u);
}

TEST(MultithreadingVerify, ConcurrentCompetitionMultiComponentSuccessAndFailure) {
    const std::vector<std::string> expected_order = {"aaa_algo.so", "mmm_algo.so", "zzz_algo.so"};

    for (int repeat = 0; repeat < 3; ++repeat) {
        ScratchDir results_dir("mt_verify_results_competition_mixed_" + std::to_string(repeat));
        const std::size_t ran = runCompetitionScratchMixed(std::to_string(repeat), results_dir.path());

        EXPECT_EQ(ran, 3u);
        for (const std::string stem : {"aaa_algo", "mmm_algo", "zzz_algo"}) {
            EXPECT_TRUE(std::filesystem::exists(results_dir.path() / ("simulation_output_" + stem + ".yaml")));
        }
        EXPECT_FALSE(std::filesystem::exists(results_dir.path() / "simulation_output_garbage_algo.yaml"));

        const YAML::Node report =
            YAML::LoadFile((results_dir.path() / "competitive_report.yaml").string())["competitive_report"];

        const std::vector<std::string> errors = extractErrors(report);
        EXPECT_EQ(errors.size(), 1u);
        EXPECT_NE(std::find(errors.begin(), errors.end(), "garbage_algo.so"), errors.end());

        // All three healthy copies are byte-identical .so files run against the same
        // composition, so they score identically -- writeCompetitiveReport's stable_sort on
        // score/steps therefore leaves them in their original insertion order, which is exactly
        // the alphabetical-by-filename processing order (outcomes is indexed by the
        // already-sorted list, regardless of which worker thread finished which component
        // first, or which thread's fetch_add claimed which index). Repeating against fresh
        // scratch setups each time confirms this ordering isn't a fluke of one particular
        // scheduling.
        std::vector<std::string> order;
        for (const CompetitiveResultEntry& entry : extractCompetitiveResultsSummary(report)) {
            order.push_back(entry.algorithm);
        }
        EXPECT_EQ(order, expected_order);
    }
}
