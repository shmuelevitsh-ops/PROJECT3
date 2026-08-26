#pragma once

#include <cstddef>
#include <functional>
#include <optional>

namespace simulator {

// Runs a fixed number of indexed tasks, either sequentially on the calling thread
// or spread across worker threads.
// Knows nothing about what a task does: it only distributes task indices.
class ParallelExecutor {
public:
    // num_threads is the thread count requested on the command line.
    // No value, or a value of 1 or less, means run sequentially.
    explicit ParallelExecutor(std::optional<int> num_threads);

    // Runs task(index) once for every index in [0, item_count).
    // Returns only after every task has finished.
    // An exception escaping task is swallowed, so one failing task cannot stop the others.
    void run(std::size_t item_count, const std::function<void(std::size_t)>& task) const;

private:
    // Determines how many worker threads to use for work_items tasks.
    // Returns 0 when the work should run sequentially on the calling thread.
    [[nodiscard]] std::size_t computeWorkerCount(std::size_t work_items) const;

    std::optional<int> num_threads_;
};

} // namespace simulator
