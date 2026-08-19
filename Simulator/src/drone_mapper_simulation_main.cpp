#include <Simulator/CerrContextGuard.h>
#include <Simulator/CliOptions.h>
#include <Simulator/SimulatorRunner.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <variant>
#include <optional>

namespace {

// Creates a UTC timestamp for result directory names.
// Name collisions are handled separately when the directory is created.
std::string resultsTimestamp() {
    // Convert to calendar time for formatting the date and time.
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
    gmtime_r(&now_time_t, &utc_tm);

    std::ostringstream out;
    out << std::put_time(&utc_tm, "%Y%m%d_%H%M%S");
    return out.str();
}

// Builds the results directory path for the selected run mode.
// The parent directory differs between comparative and competition modes.
std::filesystem::path baseResultsDir(
    const std::variant<simulator::ComparativeOptions, simulator::CompetitionOptions>& options) {
     if (std::holds_alternative<simulator::ComparativeOptions>(options)) {
        const auto& comparative = std::get<simulator::ComparativeOptions>(options);
        return comparative.mission_control_folder / ("comparative_results_" + resultsTimestamp());
    }

    const auto& competition = std::get<simulator::CompetitionOptions>(options);
    return competition.algorithms_folder / ("competition_" + resultsTimestamp());
}

// Creates the results directory.
// If the base name already exists, tries suffixes such as _2, _3, and so on.
std::optional<std::filesystem::path> createUniqueResultsDir(
    const std::filesystem::path& base_results_dir) {

    std::error_code ec;
    std::filesystem::path results_dir = base_results_dir;

    for (int suffix = 2;; ++suffix) {
        const bool created = std::filesystem::create_directories(results_dir, ec);

        if (ec) {
            std::cerr << "simulator_322889890_315113738: failed to create results directory '"
                      << results_dir.string() << "': " << ec.message() << '\n';
            return std::nullopt;
        }

        if (created) {
            return results_dir;
        }

        results_dir = base_results_dir;
        results_dir += "_" + std::to_string(suffix);
    }
}

// Prepares the output and error logging, then runs the selected simulation mode.
// Returns 0 on success and 1 on setup or unrecoverable errors.
int runSimulator(
    const std::variant<simulator::ComparativeOptions, simulator::CompetitionOptions>& options) {

    // 1. Create a unique results directory.
    const std::filesystem::path base_results_dir = baseResultsDir(options);

    const auto results_dir_opt = createUniqueResultsDir(base_results_dir);
    if (!results_dir_opt) {
        return 1; // error already printed by createUniqueResultsDir
    }

    const std::filesystem::path& results_dir = *results_dir_opt;

    // 2. Create error.log and direct std::cerr output to it.
    const std::filesystem::path error_log_path = results_dir / "error.log";
    std::ofstream error_log(error_log_path);
    if (!error_log) {
        std::cerr << "simulator_322889890_315113738: failed to open error log file '"
                  << error_log_path.string() << "'\n";

        std::error_code cleanup_ec;
        std::filesystem::remove(results_dir, cleanup_ec);

        return 1;
    }

    // From this point, errors written to std::cerr are saved in error.log.
    const simulator::CerrSinkGuard cerr_guard(error_log.rdbuf());

    // 3. Run the simulator according to the selected mode.
    try {
        std::size_t component_count = 0;

        if (std::holds_alternative<simulator::ComparativeOptions>(options)) {
            const auto& comparative = std::get<simulator::ComparativeOptions>(options);
            component_count = simulator::runComparative(comparative, results_dir);
        } else {
            const auto& competition = std::get<simulator::CompetitionOptions>(options);
            component_count = simulator::runCompetition(competition, results_dir);
        }

        std::cout << "simulator_322889890_315113738: ran " << component_count << " component(s).\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "simulator_322889890_315113738: unrecoverable error: "
                  << e.what() << '\n';
        return 1;
    }
}

} // namespace


int main(int argc, char** argv) {
    const std::optional<std::variant<simulator::ComparativeOptions, simulator::CompetitionOptions>> options =
        simulator::parseCliOptions(argc, argv);
    if (!options) {
        return 1; // usage/error already printed by parseCliOptions
    }
    return runSimulator(*options);
}
