#include <Simulator/SimulatorRunner.h>

#include <Simulator/CerrContextGuard.h>
#include <Simulator/ConfigLoader.h>
#include <Simulator/Registrar.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationOutputWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <algorithm>
#include <atomic>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

namespace simulator {

namespace {

// Sorts component libraries by filename to keep processing and report order deterministic.
std::vector<std::filesystem::path> sortedByFilename(std::vector<std::filesystem::path> libraries) {
    std::sort(libraries.begin(), libraries.end(), [](const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
        return lhs.filename().string() < rhs.filename().string();
    });
    return libraries;
}

// Stores the result of running one component.
// totals remains empty if that component fails.
struct ComponentOutcome {
    std::string component_name;
    std::optional<ComponentRunTotals> totals; // set only if this component ran successfully
};

// Runs process(index) for every item.
// Uses the calling thread when worker_count is 0, otherwise distributes work among worker threads.
// Template allows process to be any function or lambda.
// F&& accepts the given function or lambda directly, without an unnecessary copy.
template <typename F>
void runIndexed(std::size_t item_count, std::size_t worker_count, F&& process) {
    if (worker_count == 0) {
        for (std::size_t i = 0; i < item_count; ++i) {
            try {
            process(i);
            } catch (...) {
                // Isolate an unexpected failure to this task so later tasks can still run.
            }
        }
        return;
    }
    // Index of the next task not yet assigned to a worker.
    // Atomic access prevents a data race between workers.
    std::atomic<std::size_t> next{0};
    std::vector<std::jthread> workers;
    workers.reserve(worker_count);
    for (std::size_t t = 0; t < worker_count; ++t) {
        // Creates a worker thread and stores it in the vector.
        // Each worker handles multiple tasks instead of creating a new thread per task.
        workers.emplace_back([&next, item_count, &process]() {
            // This lambda is the function executed by each worker thread.
            for (;;) {
                // Atomically take the next (unique) task index.
                const std::size_t i = next.fetch_add(1);
                if (i >= item_count) {
                    break;
                }
                try {
                    process(i);
                } catch (...) {
                        // Prevent an unexpected exception from escaping the worker thread
                        // and terminating the entire process.
                    }
            }
        });
    }
}

// Runs one loaded component, writes its output, and stores its totals.
// On failure, logs the error and leaves outcome.totals empty.
void runOneComponent(const std::string& component_name, const std::string& component_stem,
                     const ParsedComposition& parsed, const std::filesystem::path& results_dir,
                     const common::MappingAlgorithmFactory& mapping_algorithm_factory,
                     const common::MissionControlFactory& mission_control_factory, bool verbose,
                     ComponentOutcome& outcome) {
    try {
        auto factory_impl = std::make_unique<SimulationRunFactoryImpl>(
            mapping_algorithm_factory, mission_control_factory, verbose);
        SimulationManager manager{std::move(factory_impl)};

        const types::SimulationManagerReport report =
            manager.run(parsed.composition, results_dir / component_stem, parsed.file_paths);

        writeSimulationOutput(report, parsed.file_paths,
                              results_dir / ("simulation_output_" + component_stem + ".yaml"));
        outcome.totals = computeComponentTotals(component_name, report);
    } catch (const std::exception& e) {
        std::cerr << "component " << component_name << " failed during simulation/output: " << e.what() << '\n';
    }
}

// Separates component outcomes into successful totals and failed component names.
void aggregateOutcomes(std::vector<ComponentOutcome>& outcomes, std::vector<ComponentRunTotals>& totals,
                       std::vector<std::string>& failures) {
    for (ComponentOutcome& outcome : outcomes) {
        if (outcome.totals) {
            totals.push_back(std::move(*outcome.totals));
        } else {
            failures.push_back(outcome.component_name);
        }
    }
}

} // namespace

// Determines how many worker threads to use.
// Returns 0 when the work should run sequentially on the calling thread.
std::size_t computeWorkerCount(const std::optional<int>& num_threads, std::size_t work_items) {
    if (!num_threads.has_value() || *num_threads <= 1 || work_items <= 1) {
        return 0;
    }
    return std::min<std::size_t>(static_cast<std::size_t>(*num_threads), work_items);
}

std::size_t runComparative(const ComparativeOptions& options, const std::filesystem::path& results_dir) {
    // Parse the shared simulation composition once for all MissionControls.
    const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

    // Load the fixed Algorithm once. All MissionControls will run against this factory.
    const common::MappingAlgorithmFactory mapping_algorithm_factory =
        Registrar::instance().loadMappingAlgorithm(options.algorithm_so_file);

    // The MissionControls are the components being compared in comparative mode.
    const std::vector<std::filesystem::path> mission_control_libraries =
        sortedByFilename(options.mission_control_libraries);

    // Preallocate one result slot per MissionControl so workers can write by index.
    std::vector<ComponentOutcome> outcomes(mission_control_libraries.size());
    const std::size_t worker_count =
        computeWorkerCount(options.num_threads, mission_control_libraries.size());
    
    // Process every MissionControl, sequentially or with workers according to worker_count.
    runIndexed(mission_control_libraries.size(), worker_count, [&](std::size_t index) {
        // [&] gives the lambda reference access to the surrounding scope variables it uses,
        // so runIndexed only needs to pass the current index.
        // Each call processes one MissionControl selected by its index.
        const std::filesystem::path& library_path = mission_control_libraries[index];
        // Store the component identity for reporting success or failure.
        outcomes[index].component_name = library_path.filename().string();
        const std::string& component_name = outcomes[index].component_name;
        const std::string component_stem = library_path.stem().string();

        // Add this component's name to every error logged by this thread
        const CerrContextGuard component_guard("component=" + component_name);

        // Load the current MissionControl independently from the other components.
        common::MissionControlFactory mission_control_factory;
        try {
            mission_control_factory = Registrar::instance().loadMissionControl(library_path);
        } catch (const std::exception& e) {
            // A load failure affects only this component, so the remaining components can continue.
            std::cerr << "failed to load " << component_name << ": " << e.what() << '\n';
            return;
        }
        // Run the whole composition using the fixed Algorithm and this MissionControl.
        runOneComponent(component_name, component_stem, parsed, results_dir, mapping_algorithm_factory,
                        mission_control_factory, options.verbose, outcomes[index]);
    });

    // Split completed components into successful totals and failures.
    // All workers have finished here, so results can now be aggregated safely.
    std::vector<ComponentRunTotals> totals;
    std::vector<std::string> failures;
    aggregateOutcomes(outcomes, totals, failures);
    // Write the final report comparing all MissionControls.
    writeComparativeReport(options.simulation_composition_file, options.mission_control_folder, totals, failures,
                          results_dir / "comparative_report.yaml");

    return totals.size();
}

std::size_t runCompetition(const CompetitionOptions& options, const std::filesystem::path& results_dir) {
    // Parse the shared simulation composition once for all Algorithms.
    const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

    // Load the fixed MissionControl once. All Algorithms will run against this factory.
    const common::MissionControlFactory mission_control_factory =
        Registrar::instance().loadMissionControl(options.mission_control_so_file);
    // The Algorithms are the components being compared in competition mode.
    const std::vector<std::filesystem::path> algorithm_libraries = sortedByFilename(options.algorithm_libraries);
    // Preallocate one result slot per Algorithm so workers can write by index.
    std::vector<ComponentOutcome> outcomes(algorithm_libraries.size());
    const std::size_t worker_count = computeWorkerCount(options.num_threads, algorithm_libraries.size());

    // Process every Algorithm, sequentially or with workers according to worker_count.
    runIndexed(algorithm_libraries.size(), worker_count, [&](std::size_t index) {
        // [&] gives the lambda reference access to the surrounding scope variables it uses,
        // so runIndexed only needs to pass the current index.

        // Each call processes one Algorithm selected by its index.
        const std::filesystem::path& library_path = algorithm_libraries[index];
        // Store the component identity for reporting success or failure.
        outcomes[index].component_name = library_path.filename().string();
        const std::string& component_name = outcomes[index].component_name;
        const std::string component_stem = library_path.stem().string();
        // Add this component's name to every error logged by this thread.
        const CerrContextGuard component_guard("component=" + component_name);
        // Load the current Algorithm independently from the other components.
        common::MappingAlgorithmFactory mapping_algorithm_factory;
        try {
            mapping_algorithm_factory = Registrar::instance().loadMappingAlgorithm(library_path);
        } catch (const std::exception& e) {
            // A load failure affects only this component, so the remaining components can continue.
            std::cerr << "failed to load " << component_name << ": " << e.what() << '\n';
            return;
        }

        // Run the whole composition using this Algorithm and the fixed MissionControl.
        runOneComponent(component_name, component_stem, parsed, results_dir, mapping_algorithm_factory,
                        mission_control_factory, options.verbose, outcomes[index]);
    });

    // Split completed components into successful totals and failures.
    // All workers have finished here, so results can now be aggregated safely.
    std::vector<ComponentRunTotals> totals;
    std::vector<std::string> failures;
    aggregateOutcomes(outcomes, totals, failures);

    // Write the final report comparing all Algorithms.
    writeCompetitiveReport(options.simulation_composition_file, options.mission_control_so_file, totals, failures,
                          results_dir / "competitive_report.yaml");

    return totals.size();
}

} // namespace simulator
