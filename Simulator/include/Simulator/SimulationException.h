#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace simulator {

// Carries a stable, machine-readable error code
// alongside the human-readable message, so SimulationManager can
// populate common::types::ErrorRef without parsing exception text.
//
// Derives from std::invalid_argument so it can still be caught wherever
// std::invalid_argument is expected.
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

} // namespace simulator