#include <Simulator/CliOptions.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>

namespace simulator {

namespace {

constexpr const char* kUsage =
    "Usage:\n"
    "  simulator_322889890_315113738 -comparative simulation=<file> mission_control_folder=<dir> "
    "algorithm=<file> [num_threads=<N>] [-verbose]\n"
    "  simulator_322889890_315113738 -competition simulation=<file> mission_control=<file> "
    "algorithms_folder=<dir> [num_threads=<N>] [-verbose]\n";

void printUsage(const std::vector<std::string>& errors) {
    std::cerr << kUsage;
    for (const std::string& error : errors) {
        std::cerr << "Error: " << error << '\n';
    }
}

// "Existing, openable file" per the assignment: must be a regular file that
// can actually be opened for reading (catches both missing paths and
// permission failures).
bool isOpenableFile(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec) || ec) {
        return false;
    }
    std::ifstream file(path);
    return file.is_open();
}

// Collects all regular .so files directly under dir.
// Returns nullopt if the directory is invalid or no .so files are found.
std::optional<std::vector<std::filesystem::path>> collectSharedLibraries(const std::filesystem::path& dir) {
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec) || ec) {
        return std::nullopt;
    }

    std::vector<std::filesystem::path> libraries;
    std::filesystem::directory_iterator entry(dir, ec);
    if (ec) {
        return std::nullopt;
    }
    const std::filesystem::directory_iterator end;
    while (entry != end) {
        std::error_code stat_ec;
        const bool is_regular = entry->is_regular_file(stat_ec);
        if (!stat_ec && is_regular && entry->path().extension() == ".so") {
            libraries.push_back(entry->path());
        }
        // Skip entries that isn't a usable .so

        entry.increment(ec);
        if (ec) {
            // Abort if directory iteration itself fails.
            return std::nullopt;
        }
    }
    if (libraries.empty()) {
        return std::nullopt;
    }
    return libraries;
}

// Parses a CLI value as an integer >= 1.
// Returns nullopt if parsing fails, the value is non-positive, or trailing characters remain.
std::optional<int> parsePositiveInt(const std::string& raw) {
    if (raw.empty()) {
        return std::nullopt;
    }
    std::size_t consumed = 0;
    try {
        const int parsed = std::stoi(raw, &consumed);
        if (consumed != raw.size() || parsed < 1) {
            return std::nullopt;
        }
        return parsed;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

struct ParsedArgs {
    int comparative_count = 0;
    int competition_count = 0;
    bool verbose = false;
    std::map<std::string, std::vector<std::string>> values; // key -> every raw value seen (duplicates kept for reporting)
    std::vector<std::string> malformed;                     // tokens that are neither a mode flag, -verbose, nor key=value
};

// Converts the raw CLI arguments into a structured intermediate form (ParsedArgs).
// Detailed validation is performed in later stages.
ParsedArgs tokenize(int argc, char** argv) {
    ParsedArgs parsed;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-comparative") {
            ++parsed.comparative_count;
            continue;
        }
        if (arg == "-competition") {
            ++parsed.competition_count;
            continue;
        }
        if (arg == "-verbose") {
            parsed.verbose = true;
            continue;
        }
        const std::size_t eq_pos = arg.find('=');
        if (eq_pos == std::string::npos || eq_pos == 0) {
            parsed.malformed.push_back(arg);
            continue;
        }
        parsed.values[arg.substr(0, eq_pos)].push_back(arg.substr(eq_pos + 1));
    }
    return parsed;
}

// Validates that exactly one run mode was selected.
// Returns true for comparative, false for competition, or nullopt on error.
std::optional<bool> resolveMode(const ParsedArgs& parsed) {
    std::vector<std::string> errors;
    if (parsed.comparative_count == 0 && parsed.competition_count == 0) {
        errors.push_back("exactly one of -comparative or -competition is required");
    } else if (parsed.comparative_count + parsed.competition_count > 1) {
        errors.push_back("-comparative and -competition are mutually exclusive and may each appear at most once");
    }
    if (!errors.empty()) {
        printUsage(errors);
        return std::nullopt;
    }
    return parsed.comparative_count == 1;
}

// Validates the basic CLI structure after tokenization.
// Rejects malformed arguments and duplicate keys.
bool validateTokensAndDuplicates(const ParsedArgs& parsed) {
    std::vector<std::string> errors;
    for (const std::string& token : parsed.malformed) {
        errors.push_back("unrecognized argument '" + token + "' (expected key=value or -verbose)");
    }
    for (const auto& [key, occurrences] : parsed.values) {
        if (occurrences.size() > 1) {
            errors.push_back("argument '" + key + "' was specified more than once");
        }
    }
    if (!errors.empty()) {
        printUsage(errors);
        return false;
    }
    return true;
}

// Validates that the provided keys match the selected run mode.
// Rejects unsupported keys and missing required keys.
bool validateKeys(const ParsedArgs& parsed, bool is_comparative) {
    std::set<std::string> mandatory_keys;
    if(is_comparative){
        mandatory_keys = std::set<std::string>{"simulation", "mission_control_folder", "algorithm"};
    } else {
        mandatory_keys = std::set<std::string>{"simulation", "mission_control", "algorithms_folder"};
    }
    std::set<std::string> allowed_keys = mandatory_keys;
    allowed_keys.insert("num_threads");

    std::vector<std::string> errors;

    for (const auto& [key, occurrences] : parsed.values) {
        if (allowed_keys.find(key) == allowed_keys.end()) {
            errors.push_back("unsupported argument '" + key + "' for " +
                             std::string(is_comparative ? "-comparative" : "-competition"));
        }
    }

    for (const std::string& key : mandatory_keys) {
        if (parsed.values.find(key) == parsed.values.end()) {
            errors.push_back("missing required argument '" + key + "='");
        }
    }

    if (!errors.empty()) {
        printUsage(errors);
        return false;
    }
    return true;
}

// Parses the optional num_threads argument.
// If present but invalid, adds an error and returns nullopt.
std::optional<int> parseNumThreadsOption(const ParsedArgs& parsed, std::vector<std::string>& errors) {
    const auto it = parsed.values.find("num_threads");
    if (it == parsed.values.end()) {
        return std::nullopt;
    }
    const std::string& raw = it->second.front();
    std::optional<int> num_threads = parsePositiveInt(raw);
    if (!num_threads) {
        errors.push_back("num_threads must be an integer >= 1, got '" + raw + "'");
    }
    return num_threads;
}

// Validates the comparative-mode file and folder arguments.
// Builds and returns ComparativeOptions if all values are valid.
std::optional<ComparativeOptions> buildComparativeOptions(const ParsedArgs& parsed, std::optional<int> num_threads,
                                                            std::vector<std::string> errors) {
    const std::filesystem::path simulation = parsed.values.at("simulation").front();
    const std::filesystem::path mission_control_folder = parsed.values.at("mission_control_folder").front();
    const std::filesystem::path algorithm = parsed.values.at("algorithm").front();

    if (!isOpenableFile(simulation)) {
        errors.push_back("simulation='" + simulation.string() + "' is not an existing, openable file");
    }
    if (!isOpenableFile(algorithm)) {
        errors.push_back("algorithm='" + algorithm.string() + "' is not an existing, openable file");
    }
    std::optional<std::vector<std::filesystem::path>> libraries = collectSharedLibraries(mission_control_folder);
    if (!libraries) {
        errors.push_back("mission_control_folder='" + mission_control_folder.string() +
                         "' is not an existing, traversable directory containing at least one .so file");
    }

    if (!errors.empty()) {
        printUsage(errors);
        return std::nullopt;
    }

    ComparativeOptions options;
    options.simulation_composition_file = simulation;
    options.mission_control_folder = mission_control_folder;
    options.algorithm_so_file = algorithm;
    options.mission_control_libraries = std::move(*libraries);
    options.num_threads = num_threads;
    options.verbose = parsed.verbose;
    return options;
}

// Validates the competition-mode file and folder arguments.
// Builds and returns CompetitionOptions if all values are valid.
std::optional<CompetitionOptions> buildCompetitionOptions(const ParsedArgs& parsed, std::optional<int> num_threads,
                                                            std::vector<std::string> errors) {
    const std::filesystem::path simulation = parsed.values.at("simulation").front();
    const std::filesystem::path mission_control = parsed.values.at("mission_control").front();
    const std::filesystem::path algorithms_folder = parsed.values.at("algorithms_folder").front();

    if (!isOpenableFile(simulation)) {
        errors.push_back("simulation='" + simulation.string() + "' is not an existing, openable file");
    }
    if (!isOpenableFile(mission_control)) {
        errors.push_back("mission_control='" + mission_control.string() + "' is not an existing, openable file");
    }
    std::optional<std::vector<std::filesystem::path>> libraries = collectSharedLibraries(algorithms_folder);
    if (!libraries) {
        errors.push_back("algorithms_folder='" + algorithms_folder.string() +
                         "' is not an existing, traversable directory containing at least one .so file");
    }

    if (!errors.empty()) {
        printUsage(errors);
        return std::nullopt;
    }

    CompetitionOptions options;
    options.simulation_composition_file = simulation;
    options.mission_control_so_file = mission_control;
    options.algorithms_folder = algorithms_folder;
    options.algorithm_libraries = std::move(*libraries);
    options.num_threads = num_threads;
    options.verbose = parsed.verbose;
    return options;
}

} // namespace

// Parses and validates the CLI arguments, then builds the options for the selected mode.
std::optional<std::variant<ComparativeOptions, CompetitionOptions>>
parseCliOptions(int argc, char** argv) {
    const ParsedArgs parsed = tokenize(argc, argv);

    const std::optional<bool> is_comparative = resolveMode(parsed);
    if (!is_comparative) {
        return std::nullopt;
    }

    if (!validateTokensAndDuplicates(parsed)) {
        return std::nullopt;
    }

    if (!validateKeys(parsed, *is_comparative)) {
        return std::nullopt;
    }

    std::vector<std::string> errors;
    const std::optional<int> num_threads = parseNumThreadsOption(parsed, errors);

    if (*is_comparative) {
        return buildComparativeOptions(parsed, num_threads, std::move(errors));
    }
    return buildCompetitionOptions(parsed, num_threads, std::move(errors));
}

} // namespace simulator
