#pragma once

#include <Common/IMappingAlgorithm.h>

#include <functional>
#include <memory>

namespace common {

using MappingAlgorithmFactory =
    std::function<std::unique_ptr<IMappingAlgorithm>(MappingAlgorithmDependencies)>;

} // namespace common
