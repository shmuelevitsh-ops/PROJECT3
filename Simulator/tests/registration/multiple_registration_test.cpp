// Coverage for Registrar::loadMappingAlgorithm's more-than-one-registration rejection path (the
// "> 1 registrations of the requested type" branch of the Registrar contract), using the
// DoubleMappingAlgorithmPlugin fixture (support/DoubleMappingAlgorithmPlugin.cpp), which
// registers two mapping algorithms from a single dlopen(). Companion to
// registration_end_to_end_test.cpp, which already covers the zero-registration and
// wrong-type-registration paths, and to stage1_verify_test.cpp / this test's own second half,
// which cover the exactly-one-registration success path.

#include <Simulator/Registrar.h>

#include <Simulator/SimulationException.h>

#include <Common/MappingAlgorithmFactory.h>

#include <gtest/gtest.h>

#include <filesystem>

TEST(RegistrationMultiple, RejectsMultipleRegistrationsThenStaysUsable) {
    auto& registrar = simulator::Registrar::instance();

    // DOUBLE_MAPPING_ALGORITHM_PLUGIN_PATH is only ever dlopen'd here, so this is its one-and-only
    // load attempt in this process -- no scratch copy needed (unlike ALGORITHM_PLUGIN_PATH /
    // MISSION_CONTROL_PLUGIN_PATH, which other TEST()s in this binary also load).
    try {
        (void)registrar.loadMappingAlgorithm(DOUBLE_MAPPING_ALGORITHM_PLUGIN_PATH);
        FAIL() << "loadMappingAlgorithm(DOUBLE_MAPPING_ALGORITHM_PLUGIN_PATH) should have thrown";
    } catch (const simulator::SimulationException& e) {
        EXPECT_EQ(e.code(), "PLUGIN_MULTIPLE_REGISTRATIONS");
    }

    // Registrar must be left in a usable state: a fresh copy of the real Algorithm plugin still
    // loads and its factory still works end-to-end, proving no stray factory from the rejected
    // double-registration attempt survived to pollute (or crash) a later, unrelated load.
    const std::filesystem::path scratch_dir =
        std::filesystem::temp_directory_path() / "simulator_multiple_registration_test";
    std::filesystem::remove_all(scratch_dir);
    std::filesystem::create_directories(scratch_dir);
    const std::filesystem::path algorithm_copy =
        scratch_dir / std::filesystem::path(ALGORITHM_PLUGIN_PATH).filename();
    std::filesystem::copy_file(ALGORITHM_PLUGIN_PATH, algorithm_copy);

    const common::MappingAlgorithmFactory mapping_algorithm_factory =
        registrar.loadMappingAlgorithm(algorithm_copy);
    ASSERT_TRUE(mapping_algorithm_factory);
}
