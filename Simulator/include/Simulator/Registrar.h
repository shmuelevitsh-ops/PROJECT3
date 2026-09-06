#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// Loads plugin libraries, collects their registered factories,
// and keeps the libraries alive while those factories exist.
// Not internally thread-safe; all plugin loading and registration are completed
// sequentially before worker threads begin.
class Registrar {
public:
    static Registrar& instance();

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;

    void addMappingAlgorithm(common::MappingAlgorithmFactory factory);
    void addMissionControl(common::MissionControlFactory factory);

    // Loads library_path and returns the single factory it registers. Throws
    // SimulationException if it registers zero or more than one factory of the requested type.
    [[nodiscard]] common::MappingAlgorithmFactory loadMappingAlgorithm(
        const std::filesystem::path& library_path);
    [[nodiscard]] common::MissionControlFactory loadMissionControl(
        const std::filesystem::path& library_path);

private:
    Registrar() = default;
    ~Registrar();

    // Private RAII helper for Registrar-owned plugin handles.
    // Nested here because only Registrar should manage loaded .so lifetimes.
    // Owns a dlopen handle and closes it on destruction.
    class LibraryHandle {
    public:
        explicit LibraryHandle(const std::filesystem::path& library_path);
        ~LibraryHandle();

        LibraryHandle(const LibraryHandle&) = delete;
        LibraryHandle& operator=(const LibraryHandle&) = delete;

        LibraryHandle(LibraryHandle&& other) noexcept;
        LibraryHandle& operator=(LibraryHandle&& other) noexcept;

    private:
        void* handle_ = nullptr;
    };

    // Loads library_path and returns the single factory of type TargetFactory it registers,
    // cleaning up any incidental OtherFactory registrations along the way. Shared flow behind
    // loadMappingAlgorithm() and loadMissionControl() -- see their doc comments above for the
    // thrown-exception contract. target_before/other_before are the respective registries' sizes
    // snapshotted by the caller before constructing LibraryHandle.
    template <typename TargetFactory, typename OtherFactory>
    TargetFactory loadComponent(const std::filesystem::path& library_path,
                                std::vector<TargetFactory>& target_factories, std::size_t target_before,
                                std::vector<OtherFactory>& other_factories, std::size_t other_before,
                                const std::string& component_name, const std::string& component_name_plural);

    std::vector<common::MappingAlgorithmFactory> mapping_algorithm_factories_;
    std::vector<common::MissionControlFactory> mission_control_factories_;
    std::vector<LibraryHandle> libraries_;
};

} // namespace simulator
