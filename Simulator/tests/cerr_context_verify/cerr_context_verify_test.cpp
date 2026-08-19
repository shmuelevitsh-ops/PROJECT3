// Permanent coverage for Stage 1 of MULTI_THREADS_PLAN.md -- exercises the new
// simulator::CerrSinkGuard / simulator::CerrContextGuard pair directly: single-thread nested
// prefix formatting (unchanged from the old PrefixingStreambuf-based text format), the no-guard
// passthrough case, concurrent non-interleaving/attribution under real std::thread contention, and
// the single-active-CerrSinkGuard invariant.

#include <Simulator/CerrContextGuard.h>

#include <gtest/gtest.h>

#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

TEST(CerrContextVerify, SingleThreadNestingMatchesOldTextFormat) {
    std::ostringstream buffer;
    const simulator::CerrSinkGuard sink(buffer.rdbuf());

    {
        const simulator::CerrContextGuard component_guard("component=X");
        const simulator::CerrContextGuard sim_guard("sim=Y");
        std::cerr << "hello\n";
    }

    EXPECT_EQ(buffer.str(), "[component=X] [sim=Y] hello\n");
}

TEST(CerrContextVerify, NoPrefixOutsideAnyGuard) {
    std::ostringstream buffer;
    const simulator::CerrSinkGuard sink(buffer.rdbuf());

    std::cerr << "unprefixed\n";

    EXPECT_EQ(buffer.str(), "unprefixed\n");
}

TEST(CerrContextVerify, ConcurrentNonInterleavingAndCorrectlyAttributed) {
    std::ostringstream buffer;
    const simulator::CerrSinkGuard sink(buffer.rdbuf());

    constexpr int kThreadCount = 8;
    constexpr int kIterations = 500;

    std::vector<std::thread> threads;
    threads.reserve(kThreadCount);
    for (int i = 0; i < kThreadCount; ++i) {
        threads.emplace_back([i]() {
            const std::string label = "thread=" + std::to_string(i);
            for (int iter = 0; iter < kIterations; ++iter) {
                const simulator::CerrContextGuard guard(label);
                std::cerr << "iteration " << iter << '\n';
            }
        });
    }
    for (std::thread& t : threads) {
        t.join();
    }

    const std::string captured = buffer.str();
    std::istringstream lines(captured);
    std::string line;

    std::vector<std::vector<bool>> seen(
        kThreadCount, std::vector<bool>(kIterations, false));
    std::size_t line_count = 0;

    const std::regex line_re(R"(^\[thread=(\d+)\] iteration (\d+)$)");
    while (std::getline(lines, line)) {
        ++line_count;
        std::smatch match;
        ASSERT_TRUE(std::regex_match(line, match, line_re)) << "malformed line: " << line;

        const int thread_index = std::stoi(match[1].str());
        const int iteration = std::stoi(match[2].str());
        ASSERT_GE(thread_index, 0);
        ASSERT_LT(thread_index, kThreadCount);
        ASSERT_GE(iteration, 0);
        ASSERT_LT(iteration, kIterations);
        ASSERT_FALSE(seen[thread_index][iteration])
            << "duplicate line for thread " << thread_index << " iteration " << iteration;
        seen[thread_index][iteration] = true;
    }

    EXPECT_EQ(line_count, static_cast<std::size_t>(kThreadCount) * kIterations);
    for (int t = 0; t < kThreadCount; ++t) {
        for (int iter = 0; iter < kIterations; ++iter) {
            EXPECT_TRUE(seen[t][iter]) << "missing line for thread " << t << " iteration " << iter;
        }
    }
}

TEST(CerrContextVerify, SingleActiveGuardInvariantIsEnforced) {
    std::ostringstream first_buffer;
    const simulator::CerrSinkGuard first(first_buffer.rdbuf());

    std::ostringstream second_buffer;
    EXPECT_THROW(simulator::CerrSinkGuard second(second_buffer.rdbuf()), std::logic_error);
}

} // namespace
