#pragma once

#include <Common/MappingAlgorithmFactory.h>

#include <utility>

namespace common {

struct MappingAlgorithmRegistration {
    explicit MappingAlgorithmRegistration(MappingAlgorithmFactory factory);
};

} // namespace common

#define REGISTER_MAPPING_ALGORITHM(class_name)                                      \
    [[maybe_unused]] ::common::MappingAlgorithmRegistration register_me_##class_name{ \
        [](::common::MappingAlgorithmDependencies dependencies)                     \
            -> std::unique_ptr<::common::IMappingAlgorithm> {                       \
            return std::make_unique<class_name>(std::move(dependencies));            \
        }}
