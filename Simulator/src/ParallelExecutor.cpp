#include <Simulator/ParallelExecutor.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <utility>
#include <vector>

namespace simulator {

ParallelExecutor::ParallelExecutor(std::optional<int> num_threads) : num_threads_(std::move(num_threads)) {}

std::size_t ParallelExecutor::computeWorkerCount(std::size_t work_items) const {
    if (!num_threads_.has_value() || *num_threads_ <= 1 || work_items <= 1) {
        return 0;
    }
    return std::min<std::size_t>(static_cast<std::size_t>(*num_threads_), work_items);
}

void ParallelExecutor::run(
    std::size_t item_count,
    const std::function<void(std::size_t)>& task,
    const std::function<void(std::size_t, std::exception_ptr)>& on_failure) const {

    const std::size_t worker_count = computeWorkerCount(item_count);

    if (worker_count == 0) {
        for (std::size_t i = 0; i < item_count; ++i) {
            try {
                task(i);
            } catch (...) {
                on_failure(i, std::current_exception());
            }
        }
        return;
    }

    // Index of the next task not yet assigned to a worker.
    // Atomic access prevents a data race between workers.
    std::atomic<std::size_t> next{0};
    std::vector<std::jthread> workers;
    workers.reserve(worker_count);
    for (std::size_t t = 0; t < worker_count; ++t) {
        // Each worker repeatedly claims tasks from the shared atomic index.
        workers.emplace_back([&next, item_count, &task, &on_failure]() {
            for (;;) {
                // Atomically take the next (unique) task index.
                const std::size_t i = next.fetch_add(1);
                if (i >= item_count) {
                    break;
                }
                try {
                    task(i);
                } catch (...) {
                    on_failure(i, std::current_exception());
                }
            }
        });
    }
    // Destroying the jthreads joins all workers before run() returns.
}

} // namespace simulator
