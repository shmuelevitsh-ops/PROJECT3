#include <Simulator/CerrContextGuard.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace simulator {

namespace {

std::vector<std::string>& contextStack() {
    thread_local std::vector<std::string> stack;
    return stack;
}

std::atomic<bool> g_active{false};

} // namespace

CerrContextGuard::CerrContextGuard(const std::string& context_label) {
    contextStack().push_back(context_label);
}

CerrContextGuard::~CerrContextGuard() { contextStack().pop_back(); }

std::string CerrContextGuard::currentPrefix() {
    std::string prefix;
    for (const std::string& label : contextStack()) {
        prefix += "[" + label + "] ";
    }
    return prefix;
}

class CerrSinkGuard::ConcurrentContextStreambuf final : public std::streambuf {
public:
    explicit ConcurrentContextStreambuf(std::streambuf* dest) : dest_(dest) {}

protected:
    int overflow(int ch) override {
        if (ch == traits_type::eof()) {
            return traits_type::not_eof(ch);
        }

        std::string& line = lineBuffer();
        const char_type c = traits_type::to_char_type(ch);
        line.push_back(c);

        if (c == '\n') {
            const std::string prefix = CerrContextGuard::currentPrefix();
            {
                std::lock_guard<std::mutex> lock(write_mutex_);
                dest_->sputn(prefix.data(), static_cast<std::streamsize>(prefix.size()));
                dest_->sputn(line.data(), static_cast<std::streamsize>(line.size()));
            }
            line.clear();
        }

        return ch;
    }

private:
    static std::string& lineBuffer() {
        thread_local std::string buffer;
        return buffer;
    }

    std::streambuf* dest_;
    std::mutex write_mutex_;
};

CerrSinkGuard::CerrSinkGuard(std::streambuf* destination) {
    if (g_active.exchange(true)) {
        throw std::logic_error("CerrSinkGuard: only one instance may be active at a time");
    }
    try {
        buf_ = std::make_unique<ConcurrentContextStreambuf>(destination);
        original_ = std::cerr.rdbuf(buf_.get());
    } catch (...) {
        g_active.store(false);
        throw;
    }
}

CerrSinkGuard::~CerrSinkGuard() {
    std::cerr.rdbuf(original_);
    g_active.store(false);
}

} // namespace simulator
