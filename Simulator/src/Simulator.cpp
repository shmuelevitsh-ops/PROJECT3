#include <Simulator/Simulator.h>

#include <Simulator/CerrContextGuard.h>
#include <Simulator/ConfigLoader.h>
#include <Simulator/ParallelExecutor.h>
#include <Simulator/Registrar.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationOutputWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>
#include <exception>

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

        SimulationManager manager{std::move(factory_impl), parsed.file_paths};

        const types::SimulationManagerReport report =
            manager.run(parsed.composition, results_dir / component_stem);

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

template <typename Factory>
struct LoadedComponents {
    std::vector<std::size_t> indices;
    std::vector<Factory> factories;
};

// Loads component factories sequentially before worker threads start,
// keeping successful factories aligned with their original component indices.
// Generic template that preloads component factories on the calling thread,
// supporting both Algorithm and MissionControl factory types.
template <typename Factory, typename Loader>
LoadedComponents<Factory> preloadComponents(
    const std::vector<std::filesystem::path>& libraries,
    std::vector<ComponentOutcome>& outcomes,
    Loader&& loader) {

    LoadedComponents<Factory> loaded;
    loaded.indices.reserve(libraries.size());
    loaded.factories.reserve(libraries.size());

    for (std::size_t index = 0; index < libraries.size(); ++index) {
        const std::filesystem::path& library_path = libraries[index];
        outcomes[index].component_name = library_path.filename().string();
        const std::string& component_name = outcomes[index].component_name;

        const CerrContextGuard component_guard("component=" + component_name);

        try {
            loaded.factories.push_back(loader(library_path));
            loaded.indices.push_back(index);
        } catch (const std::exception& e) {
            std::cerr << "failed to load " << component_name
                      << ": " << e.what() << '\n';
        }
    }

    return loaded;
}

// Reports an unexpected component-level exception without stopping other components.
void reportUnexpectedComponentFailure(const std::string& component_name, std::exception_ptr exception) {
    const CerrContextGuard component_guard("component=" + component_name);
    try {
            std::rethrow_exception(exception);
        } catch (const std::exception& e) {
            std::cerr << "component " << component_name
                    << " failed unexpectedly: " << e.what() << '\n';
        } catch (...) {
            std::cerr << "component " << component_name
                    << " failed with an unknown exception\n";
        }
}   

} // namespace

Simulator::Simulator(std::variant<ComparativeOptions, CompetitionOptions> options, std::filesystem::path results_dir)
    : options_(std::move(options)), results_dir_(std::move(results_dir)) {}

std::size_t Simulator::run() const {
    if (std::holds_alternative<ComparativeOptions>(options_)) {
        return runComparative();
    }
    return runCompetition();
}

std::size_t Simulator::runComparative() const {
    const ComparativeOptions& options = std::get<ComparativeOptions>(options_);

    // Parse the shared simulation composition once for all MissionControls.
    const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

    // Load the fixed Algorithm once. All MissionControls will run against this factory.
    const common::MappingAlgorithmFactory mapping_algorithm_factory =
        Registrar::instance().loadMappingAlgorithm(options.algorithm_so_file);

    // The MissionControls are the components being compared in comparative mode.
    const std::vector<std::filesystem::path> mission_control_libraries =
        sortedByFilename(options.mission_control_libraries);

    // Preallocate one result slot per MissionControl so the preload loop below, and later the
    // workers, can write by original component index.
    std::vector<ComponentOutcome> outcomes(mission_control_libraries.size());

    // Load every MissionControl sequentially, on the calling thread, before any worker thread starts.
    // A load failure affects only that one component: its outcome stays a failure (empty totals)
    // and it is left out of the successfully loaded components below.
    const auto loaded = preloadComponents<common::MissionControlFactory>(mission_control_libraries,
                                                                        outcomes,
                                                                        [](const std::filesystem::path& path) {
                                                                            return Registrar::instance().loadMissionControl(path);
                                                                        });

    // Process every already-loaded MissionControl, sequentially or with workers according to
    // num_threads. Only successfully loaded components are handed to the executor, so it never
    // sizes its worker pool for components that failed to load and have nothing left to run.
    const ParallelExecutor executor(options.num_threads);
    
    executor.run(loaded.indices.size(), [&](std::size_t task_index) {
        // task
        // task_index is a position in the successfully loaded components;
        // index is the component's original deterministic position.
        const std::size_t index = loaded.indices[task_index];
        const std::filesystem::path& library_path = mission_control_libraries[index];
        const std::string& component_name = outcomes[index].component_name;
        const std::string component_stem = library_path.stem().string();

        // Add this component's name to every error logged by this thread
        const CerrContextGuard component_guard("component=" + component_name);

        // Run the whole composition using the fixed Algorithm and this MissionControl.
        runOneComponent(component_name, component_stem, parsed, results_dir_, mapping_algorithm_factory,
                        loaded.factories[task_index], options.verbose, outcomes[index]);
    },
    [&](std::size_t task_index, std::exception_ptr exception) {
        // on_failure
        const std::size_t index = loaded.indices[task_index];
        reportUnexpectedComponentFailure(outcomes[index].component_name, exception);
    });

    // Split completed components into successful totals and failures.
    // All workers have finished here, so results can now be aggregated safely.
    std::vector<ComponentRunTotals> totals;
    std::vector<std::string> failures;
    aggregateOutcomes(outcomes, totals, failures);
    // Write the final report comparing all MissionControls.
    writeComparativeReport(options.simulation_composition_file, options.mission_control_folder, totals, failures,
                          results_dir_ / "comparative_report.yaml");

    return totals.size();
}

std::size_t Simulator::runCompetition() const {
    const CompetitionOptions& options = std::get<CompetitionOptions>(options_);

    // Parse the shared simulation composition once for all Algorithms.
    const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

    // Load the fixed MissionControl once. All Algorithms will run against this factory.
    const common::MissionControlFactory mission_control_factory =
        Registrar::instance().loadMissionControl(options.mission_control_so_file);
    // The Algorithms are the components being compared in competition mode.
    const std::vector<std::filesystem::path> algorithm_libraries = sortedByFilename(options.algorithm_libraries);
    // Preallocate one result slot per Algorithm so the preload loop below, and later the workers,
    // can write by original component index.
    std::vector<ComponentOutcome> outcomes(algorithm_libraries.size());

    // Load every Algorithm sequentially, on the calling thread, before any worker thread starts.
    // A load failure affects only that one component: its outcome stays a failure (empty totals)
    // and it is left out of the successfully loaded components below.
    const auto loaded = preloadComponents<common::MappingAlgorithmFactory>(algorithm_libraries,
                                                                        outcomes,
                                                                        [](const std::filesystem::path& path) {
                                                                            return Registrar::instance().loadMappingAlgorithm(path);
                                                                        });

    // Process every already-loaded Algorithm, sequentially or with workers according to
    // num_threads. Only successfully loaded components are handed to the executor, so it never
    // sizes its worker pool for components that failed to load and have nothing left to run.
    const ParallelExecutor executor(options.num_threads);
    
    executor.run(loaded.indices.size(), [&](std::size_t task_index) {
        // task
        // task_index is a position in the successfully loaded components;
        // index is the component's original deterministic position.
        const std::size_t index = loaded.indices[task_index];
        const std::filesystem::path& library_path = algorithm_libraries[index];
        const std::string& component_name = outcomes[index].component_name;
        const std::string component_stem = library_path.stem().string();
        // Add this component's name to every error logged by this thread.
        const CerrContextGuard component_guard("component=" + component_name);

        // Run the whole composition using this Algorithm and the fixed MissionControl.
        runOneComponent(component_name, component_stem, parsed, results_dir_, loaded.factories[task_index],
                        mission_control_factory, options.verbose, outcomes[index]);
    },
    [&](std::size_t task_index, std::exception_ptr exception) {
        // on_failure
        const std::size_t index = loaded.indices[task_index];
        reportUnexpectedComponentFailure(outcomes[index].component_name, exception);
    });

    // Split completed components into successful totals and failures.
    // All workers have finished here, so results can now be aggregated safely.
    std::vector<ComponentRunTotals> totals;
    std::vector<std::string> failures;
    aggregateOutcomes(outcomes, totals, failures);

    // Write the final report comparing all Algorithms.
    writeCompetitiveReport(options.simulation_composition_file, options.mission_control_so_file, totals, failures,
                          results_dir_ / "competitive_report.yaml");

    return totals.size();
}

} // namespace simulator
