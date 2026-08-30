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

} // namespace simulator