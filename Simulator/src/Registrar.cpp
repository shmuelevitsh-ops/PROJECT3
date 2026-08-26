#include <Simulator/Registrar.h>

#include <Simulator/SimulationException.h>

#include <dlfcn.h>

#include <string>
#include <utility>

namespace simulator {

Registrar& Registrar::instance() {
    // Function-local static: constructed once on the first call to instance().
    // Its initialization is thread-safe (since C++11), so all callers get the same Registrar.
    static Registrar registrar;
    return registrar;
}

Registrar::~Registrar() {
    // Factories may contain code from the loaded .so files, so destroy them first.
    // Clearing libraries_ afterwards destroys the handles and safely calls dlclose.
    mapping_algorithm_factories_.clear();
    mission_control_factories_.clear();
    libraries_.clear();
}

Registrar::LibraryHandle::LibraryHandle(const std::filesystem::path& library_path) {
    // Loads the plugin and verifies that all required symbols are available.
    handle_ = dlopen(library_path.c_str(), RTLD_NOW);
    if (handle_ == nullptr) {
        throw SimulationException("PLUGIN_LOAD_FAILED", dlerror());
    }
}

Registrar::LibraryHandle::~LibraryHandle() {
    // Only the current owner closes the library handle.
    if (handle_ != nullptr) {
        dlclose(handle_);
    }
}

// Move constructor: Transfer handle ownership and leave the source empty.
Registrar::LibraryHandle::LibraryHandle(LibraryHandle&& other) noexcept
    : handle_(std::exchange(other.handle_, nullptr)) {}

// Move assignment: Release the current handle, then take ownership from the source.
Registrar::LibraryHandle& Registrar::LibraryHandle::operator=(LibraryHandle&& other) noexcept {
    if (this != &other) {
        if (handle_ != nullptr) {
            dlclose(handle_);
        }
        handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
}

void Registrar::addMappingAlgorithm(common::MappingAlgorithmFactory factory) {
    mapping_algorithm_factories_.push_back(std::move(factory));
}

void Registrar::addMissionControl(common::MissionControlFactory factory) {
    mission_control_factories_.push_back(std::move(factory));
}

common::MappingAlgorithmFactory Registrar::loadMappingAlgorithm(
    const std::filesystem::path& library_path) {
    // Remember the registry sizes so we can detect what this .so registers.
    const std::size_t mapping_before = mapping_algorithm_factories_.size();
    const std::size_t mission_before = mission_control_factories_.size();

    // Constructing LibraryHandle calls dlopen(); while the .so is being loaded,
    // its global registration object is constructed and registers the factory here.
    LibraryHandle library(library_path);
    // Count how many factories of the requested type were added by this load.
    const std::size_t mapping_registered = mapping_algorithm_factories_.size() - mapping_before;

    if (mapping_registered == 0) {
        // No mapping factory was registered. Remove any wrong-type registrations
        // before the local LibraryHandle unloads the .so.
        mission_control_factories_.resize(mission_before);
        throw SimulationException("PLUGIN_NOT_REGISTERED",
                                   library_path.string() + " did not register a mapping algorithm");
    }

    if (mapping_registered > 1) {
        // More than one mapping factory was registered. Erase every factory this failed load
        // added (not just the extras) along with any wrong-type registrations, before the local
        // LibraryHandle unloads the .so, so no std::function closing over this .so's code survives.
        mapping_algorithm_factories_.resize(mapping_before);
        mission_control_factories_.resize(mission_before);
        throw SimulationException("PLUGIN_MULTIPLE_REGISTRATIONS",
                                   library_path.string() + " registered " +
                                       std::to_string(mapping_registered) +
                                       " mapping algorithms (expected exactly 1)");
    }

    // Remove any MissionControl factories accidentally registered by this .so.
    mission_control_factories_.resize(mission_before);
    // Keep the .so loaded while its registered factory is still stored.
    libraries_.push_back(std::move(library));
    // Exactly one factory was registered, and push_back placed it at the end.
    return mapping_algorithm_factories_.back();
}

common::MissionControlFactory Registrar::loadMissionControl(
    const std::filesystem::path& library_path) {
    // Remember the registry sizes so we can detect what this .so registers.
    const std::size_t mapping_before = mapping_algorithm_factories_.size();
    const std::size_t mission_before = mission_control_factories_.size();

    // Constructing LibraryHandle calls dlopen(); while the .so is being loaded,
    // its global registration object is constructed and registers the factory here.
    LibraryHandle library(library_path);
    // Count how many factories of the requested type were added by this load.
    const std::size_t mission_registered = mission_control_factories_.size() - mission_before;

    if (mission_registered == 0) {
        // No mission factory was registered. Remove any wrong-type registrations
        // before the local LibraryHandle unloads the .so.
        mapping_algorithm_factories_.resize(mapping_before);
        throw SimulationException("PLUGIN_NOT_REGISTERED",
                                   library_path.string() + " did not register a mission control");
    }

    if (mission_registered > 1) {
        // More than one mission factory was registered. Erase every factory this failed load
        // added (not just the extras) along with any wrong-type registrations, before the local
        // LibraryHandle unloads the .so, so no std::function closing over this .so's code survives.
        mission_control_factories_.resize(mission_before);
        mapping_algorithm_factories_.resize(mapping_before);
        throw SimulationException("PLUGIN_MULTIPLE_REGISTRATIONS",
                                   library_path.string() + " registered " +
                                       std::to_string(mission_registered) +
                                       " mission controls (expected exactly 1)");
    }

    // Remove any MappingAlgorithm factories accidentally registered by this .so.
    mapping_algorithm_factories_.resize(mapping_before);
    // Keep the .so loaded while its registered factory is still stored.
    libraries_.push_back(std::move(library));
    // Exactly one factory was registered, and push_back placed it at the end.
    return mission_control_factories_.back();
}

} // namespace simulator
