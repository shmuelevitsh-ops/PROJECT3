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

std::size_t Simulator::runComparative(const ComparativeOptions& options,
                                      const std::filesystem::path& results_dir) const {
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

    // Process every MissionControl, sequentially or with workers according to num_threads.
    const ParallelExecutor executor(options.num_threads);
    executor.run(mission_control_libraries.size(), [&](std::size_t index) {
        // [&] gives the lambda reference access to the surrounding scope variables it uses,
        // so the executor only needs to pass the current index.
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

std::size_t Simulator::runCompetition(const CompetitionOptions& options,
                                      const std::filesystem::path& results_dir) const {
    // Parse the shared simulation composition once for all Algorithms.
    const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

    // Load the fixed MissionControl once. All Algorithms will run against this factory.
    const common::MissionControlFactory mission_control_factory =
        Registrar::instance().loadMissionControl(options.mission_control_so_file);
    // The Algorithms are the components being compared in competition mode.
    const std::vector<std::filesystem::path> algorithm_libraries = sortedByFilename(options.algorithm_libraries);
    // Preallocate one result slot per Algorithm so workers can write by index.
    std::vector<ComponentOutcome> outcomes(algorithm_libraries.size());

    // Process every Algorithm, sequentially or with workers according to num_threads.
    const ParallelExecutor executor(options.num_threads);
    executor.run(algorithm_libraries.size(), [&](std::size_t index) {
        // [&] gives the lambda reference access to the surrounding scope variables it uses,
        // so the executor only needs to pass the current index.

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
