#include <Simulator/CerrContextGuard.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace simulator {

namespace {

// Returns the context-label stack of the calling thread.
// Each thread keeps its own stack between calls.
std::vector<std::string>& contextStack() {
    thread_local std::vector<std::string> stack;
    return stack;
}

// Tracks whether a CerrSinkGuard is currently active.
std::atomic<bool> g_active{false};

} // namespace

// Adds a context label to the calling thread's stack.
CerrContextGuard::CerrContextGuard(const std::string& context_label) {
    contextStack().push_back(context_label);
}

// Removes the context label added by this guard.
CerrContextGuard::~CerrContextGuard() { contextStack().pop_back(); }

// Builds the prefix from all active context labels of the calling thread.
std::string CerrContextGuard::currentPrefix() {
    std::string prefix;
    for (const std::string& label : contextStack()) {
        prefix += "[" + label + "] ";
    }
    return prefix;
}

// Receives std::cerr output, builds complete lines per thread,
// adds the thread's context prefix, and writes them safely to the destination.
class CerrSinkGuard::ConcurrentContextStreambuf final : public std::streambuf {
public:
    // Stores the destination stream buffer that will receive complete messages.
    explicit ConcurrentContextStreambuf(std::streambuf* dest) : dest_(dest) {}

private:
    // Handles each character written through std::cerr and writes a complete line
    // to the destination when a newline is reached.
    int overflow(int ch) override {
        // EOF is not a real character, so there is nothing to buffer.
        if (ch == traits_type::eof()) {
            return traits_type::not_eof(ch);
        }

        // Each thread builds its own line independently.
        std::string& line = lineBuffer();
        const char_type c = traits_type::to_char_type(ch); //convert int to character
        line.push_back(c);

        // Write only complete lines so messages from different threads do not interleave.
        if (c == '\n') {
            const std::string prefix = CerrContextGuard::currentPrefix();
            {
                // Lock the shared destination so one complete message is written at a time.
                std::lock_guard<std::mutex> lock(write_mutex_);

                // sputn writes a sequence of characters directly to the destination stream buffer.
                dest_->sputn(prefix.data(), static_cast<std::streamsize>(prefix.size()));
                dest_->sputn(line.data(), static_cast<std::streamsize>(line.size()));
            } // lock destroyed -> mutex unlocked

            // Prepare this thread's buffer for the next line.
            line.clear();
        }

        return ch;
    }

    // Returns the calling thread's own line buffer.
    static std::string& lineBuffer() {
        thread_local std::string buffer;
        return buffer;
    }

    std::streambuf* dest_;  // Final destination, normally error_log.rdbuf().
    std::mutex write_mutex_; // Protects complete-line writes to the shared destination.
};

// Installs the custom stream buffer on std::cerr.
CerrSinkGuard::CerrSinkGuard(std::streambuf* destination) {
    // Allow only one process-wide cerr sink at a time.
    if (g_active.exchange(true)) {
        throw std::logic_error("CerrSinkGuard: only one instance may be active at a time");
    }
    try {
        // Create the custom stream buffer that writes to the given destination.
        buf_ = std::make_unique<ConcurrentContextStreambuf>(destination);
        // Install our buffer on std::cerr and save the previous one to put it back later.
        original_ = std::cerr.rdbuf(buf_.get());
    } catch (...) {
        // Mark the sink as inactive if setup fails.
        g_active.store(false);
        throw;
    }
}

// Put back std::cerr's previous buffer and mark the sink as inactive.
CerrSinkGuard::~CerrSinkGuard() {
    std::cerr.rdbuf(original_);
    g_active.store(false);
}

} // namespace simulator
