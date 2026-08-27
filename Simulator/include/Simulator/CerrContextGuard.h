#pragma once

#include <memory>
#include <streambuf>
#include <string>

namespace simulator {

// Redirects std::cerr to a thread-safe sink that prefixes each line
// with the calling thread's active context labels.
class CerrSinkGuard {
public:
    explicit CerrSinkGuard(std::streambuf* destination);
    ~CerrSinkGuard();

    CerrSinkGuard(const CerrSinkGuard&) = delete;
    CerrSinkGuard& operator=(const CerrSinkGuard&) = delete;

private:
    class ConcurrentContextStreambuf; // defined in the .cpp
    std::unique_ptr<ConcurrentContextStreambuf> buf_;
    std::streambuf* original_;
};

// Adds a context label to the calling thread for the guard's lifetime.
// Nested guards produce nested prefixes.
class CerrContextGuard {
public:
    explicit CerrContextGuard(const std::string& context_label);
    ~CerrContextGuard();

    CerrContextGuard(const CerrContextGuard&) = delete;
    CerrContextGuard& operator=(const CerrContextGuard&) = delete;

    // Builds the current thread's combined context prefix.
    [[nodiscard]] static std::string currentPrefix();
};

} // namespace simulator
