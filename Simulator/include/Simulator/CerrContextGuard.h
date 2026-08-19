#pragma once

#include <memory>
#include <streambuf>
#include <string>

namespace simulator {

// Installs the single, process-wide, thread-safe std::cerr sink for its lifetime, restoring
// std::cerr's previous streambuf on destruction. Construct exactly once, on whichever thread owns
// std::cerr's lifetime for this run (main(), or a test's setup code), strictly before any other
// thread that might write to std::cerr is started, and keep it alive until every such thread has
// been joined. CerrContextGuard's per-thread context labels (below) only take effect while a
// CerrSinkGuard is alive somewhere on the call stack; without one, std::cerr behaves exactly as if
// this whole mechanism didn't exist (plain, unprefixed passthrough is NOT provided -- a
// CerrSinkGuard must always be installed before this codebase writes to std::cerr for prefixes to
// appear; see main()'s existing error_log setup in drone_mapper_simulation_main.cpp).
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

// A pure thread-local label stack push/pop -- does not touch std::cerr or any streambuf. Every
// guard alive on the calling thread's stack contributes its label to that thread's current line
// prefix (see currentPrefix()), which CerrSinkGuard's installed streambuf applies to every line
// written from that thread while a CerrSinkGuard is active somewhere in the process.
class CerrContextGuard {
public:
    explicit CerrContextGuard(const std::string& context_label);
    ~CerrContextGuard();

    CerrContextGuard(const CerrContextGuard&) = delete;
    CerrContextGuard& operator=(const CerrContextGuard&) = delete;

    // Returns "[label1] [label2] ... " for every guard currently alive on the CALLING thread's
    // stack, outermost first, or "" if none. Used internally by
    // CerrSinkGuard::ConcurrentContextStreambuf; exposed so it's independently unit-testable.
    [[nodiscard]] static std::string currentPrefix();
};

} // namespace simulator
