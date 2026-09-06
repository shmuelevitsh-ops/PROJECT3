#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace simulator {

// invalid_argument with an additional machine-readable error code.
class SimulationException : public std::invalid_argument {
public:
    SimulationException(std::string code, const std::string& message)
        : std::invalid_argument(message), code_(std::move(code)) {}

    [[nodiscard]] const std::string& code() const {
        return code_;
    }

private:
    std::string code_;
};

// Which injected factory a ComponentConstructionException came from. Comparative mode evaluates
// MissionControl (MappingAlgorithm is the fixed/shared component); competition mode evaluates
// MappingAlgorithm (MissionControl is fixed/shared). SimulationManager uses this tag -- never
// string-parsing of the exception's message -- to tell a failure of the component it is currently
// evaluating apart from a failure of the fixed component (see SimulationManager.cpp).
enum class ComponentKind { MappingAlgorithm, MissionControl };

[[nodiscard]] inline const char* toString(ComponentKind kind) {
    return kind == ComponentKind::MappingAlgorithm ? "mapping algorithm" : "mission control";
}

// Thrown when a component's injected factory (MappingAlgorithm or MissionControl) fails to
// construct its pluggable instance for one particular scenario. Since the factory receives
// run-specific dependencies, it may fail for one scenario while succeeding for another -- this is
// still just a per-run failure (scored -1), not automatically a component-level one. Only when
// SimulationManager::run() has attempted construction of the evaluated component (see kind())
// across the whole composition and every such attempt failed does it signal a component-level
// failure, by throwing this same exception type once more at the end of run().
class ComponentConstructionException : public std::runtime_error {
public:
    ComponentConstructionException(ComponentKind kind, const std::string& message)
        : std::runtime_error(message), kind_(kind) {}

    [[nodiscard]] ComponentKind kind() const {
        return kind_;
    }

private:
    ComponentKind kind_;
};

} // namespace simulator