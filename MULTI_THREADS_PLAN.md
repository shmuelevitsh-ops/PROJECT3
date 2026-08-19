# Multithreading — Implementation Plan

**Read `RULES.md` before every stage below.** This plan is self-contained: it does not assume
you have read the assignment PDF/DOCX or any course forum thread. Every requirement referenced
here is restated in full below.

## Goal

Add multithreading to the already-working, single-threaded comparative/competition orchestration
in `Simulator/src/SimulatorRunner.cpp` (`runComparative`/`runCompetition`), so that independent
**components** (one `MissionControl_*.so` in comparative mode, one `Algorithm_*.so` in competition
mode) run concurrently across worker threads, honoring the `num_threads` CLI argument that is
already parsed and validated (`Simulator/include/Simulator/CliOptions.h`'s
`ComparativeOptions::num_threads` / `CompetitionOptions::num_threads`, both `std::optional<int>`)
but currently unused (confirmed by inspection — grep for `num_threads` outside `CliOptions.*`
and its tests turns up nothing).

**Concurrency unit — final, not open for reinterpretation.** One work item is one whole component:
one `MissionControl_*.so` in comparative mode, one `Algorithm_*.so` in competition mode — the
existing `mission_control_libraries` / `algorithm_libraries` loop in `SimulatorRunner.cpp`. Worker
threads pull the next available component job from that list as they become free; a worker is not
permanently bound to one `.so` — with fewer workers than components, each worker processes several
jobs in turn over its lifetime.

The simulation/mission/drone/lidar combinations inside **one** component's composition sweep
(`SimulationManager::runInternal` in `Simulator/src/SimulationManager.cpp`) stay sequential and are
never distributed across worker threads — do not parallelize inside a single component's run.

Direct consequence: if `mission_control_folder` (or `algorithms_folder`) contains exactly **one**
relevant `.so`, the run is fully sequential — no worker thread is spawned — regardless of
`num_threads` and regardless of how many internal simulation/mission/drone/lidar combinations that
one component's sweep contains (see the worker-count derivation below).

This granularity is a deliberate project decision, made after reviewing the alternative of also
distributing the internal combinations, and matches the pattern used in the course's own
`recitation_10_source/` demo (worker threads claim independent `.so` paths, not the work inside
each one). Do not revisit it while implementing this plan.

**Hard requirements carried over verbatim from the assignment (already resolved, not open
questions):**
- `num_threads` missing or `1` → single thread (main thread only) — no worker thread spawned.
- `num_threads >= 2` → `<num_threads>` is the requested count of **additional** worker threads
  (i.e. *not* counting the main thread). The main thread blocking in a join on all workers is
  fine and expected.
- The total thread count (main + workers) must never be exactly 2.
- Never open a worker thread that would have nothing to do — the actual worker count must be
  capped by the amount of independent work actually available, not just by the requested number.
- Prefer no locking; where shared mutable state genuinely requires a lock, use one.
- Creating an Algorithm/MissionControl instance via its factory is cheap; never cache/reuse
  instances (already how `SimulationRunFactoryImpl::create` works today — unaffected by this
  plan).
- Every `.so` file needed for the run is loaded once and kept loaded for the process lifetime
  (already how `Registrar` works today — the "load once, unload if idle" alternative is an
  optional bonus this plan does not attempt).
- Must preserve, byte-for-byte, everything the current sequential implementation already produces
  when `num_threads` is absent or `1`: file names, directory layout, YAML content and key order,
  `comparative_report.yaml`/`competitive_report.yaml` `results_summary`/`errors` ordering, and
  `error.log` content/format **including line order** — the sequential (`worker_count == 0`) code
  path is a mechanical relocation of today's existing loop (see Stage 2), not a rewrite, so its
  output must be indistinguishable from today's in every respect, ordering included.
- Concurrent runs (`num_threads >= 2`) must produce output that is **structurally and semantically
  identical** to the sequential run for the same inputs for everything *except* `error.log`'s line
  order (see the dedicated clarification immediately below): the same files must be created, the
  same YAML content, and the same deterministic `results_summary`/`errors` order — derived from
  alphabetical-by-filename component order and reassembled from preallocated per-component result
  slots only after every worker thread has joined (see Stage 2), never from whatever order threads
  happened to finish in.

**`error.log` line ordering under concurrency — resolved explicitly, not left to interpretation:**
Across *different components*, `error.log` line **order is not required to match the sequential
run's order** once `num_threads >= 2`, and is expected to vary from run to run depending on
wall-clock thread scheduling — components finish in whatever order their work actually completes,
not in alphabetical component order. This is a deliberate, different requirement from the YAML
reports' ordering above, which *is* fixed and deterministic regardless of thread scheduling (because
it's reassembled from indexed slots after joining, not appended as-completed). What concurrent
execution *is* required to preserve — and what Stage 1's logging redesign exists specifically to
guarantee — are three properties that are orthogonal to line order:
1. **Completeness** — every message the sequential run would log for a given set of inputs is
   still logged exactly once under concurrency (nothing lost, nothing duplicated).
2. **Non-interleaving** — no single line in `error.log` is ever a corrupted mixture of bytes from
   two different threads' messages; each line, once written, is exactly one complete message from
   exactly one write.
3. **Correct attribution** — every line's `[component=...] [sim=... mission=... drone=...
   lidar=...]` context prefix correctly identifies the component/run that actually produced that
   line, on whichever thread produced it, regardless of what any other thread was doing at the
   same time.
Do not write a test, or accept a design, that asserts `error.log` is line-for-line identical
between a sequential and a concurrent run of the same inputs — that comparison is not meaningful
under concurrency and is expected to fail. Where a test needs to compare `error.log` content across
a sequential and a concurrent run of the same inputs, compare it as an unordered multiset of
expected messages (e.g. "which components logged a load failure, and with what text," irrespective
of which line came first), never by line position.

### The one non-obvious derivation this plan makes (stated so you don't have to re-derive it)

This derivation operates entirely *within* the component-level granularity confirmed as settled
above — it is about how many worker threads to spawn for a given **count of components**, never
about redefining what counts as one job. Combining "total thread count is never exactly 2" with
"never open a worker thread with nothing to do": if only **one** component needs processing (e.g.
a `mission_control_folder` containing
exactly one `.so`), spawning a single worker thread for it would make total thread count
`1 (main) + 1 (worker) = 2`, which is explicitly disallowed — even though naively capping
`min(requested_num_threads, work_items)` would yield `1` worker in that case. The correct
resolution is: **whenever the capped worker count would come out to exactly `1`, use `0` instead**
(run on the main thread directly, identical to the `num_threads` absent/`1` case). This never
loses parallelism that was actually available (there was only one job anyway, nothing to overlap),
and it is the only reading that satisfies both explicit constraints simultaneously. See
`computeWorkerCount` in Stage 2 below — this is not left as an implementation choice.

There is no other unresolved ambiguity in the threading model itself; see "Shared-state audit
already performed" below for the parts of this codebase that needed inspection (not
interpretation) to confirm they're safe to run concurrently.

---

## Shared-state audit already performed (do not re-litigate, but do spot-check the two flagged items)

Read directly, not assumed, before writing this plan:

- **`Simulator/include/Simulator/Registrar.h` + `.cpp`** — the singleton that owns every
  `dlopen`ed library handle and the factories its `REGISTER_*` macros registered.
  `loadMappingAlgorithm`/`loadMissionControl` already take `std::lock_guard<std::mutex>
  lock(load_mutex_)` around the entire load-and-check body and return a **copy** of the single
  registered `Factory` rather than a reference into internal storage. **Already safe for
  concurrent calls from multiple worker threads — no change needed.**
- **`Simulator/include/Simulator/CerrContextGuard.h` + `.cpp`** — **not safe**, and already
  flagged in its own header comment: `std::cerr.rdbuf()` is one process-wide mutable pointer with
  no synchronization, and today's `PrefixingStreambuf`/`CerrContextGuard` pair works only because
  every guard in the process today is installed/torn down strictly sequentially. This is the
  subject of Stage 1 below.
- **`Simulator/src/SimulatorRunner.cpp`'s `totals`/`failures` accumulation** (currently
  `std::vector<ComponentRunTotals>& totals, std::vector<std::string>& failures` appended to via
  `push_back` once per loop iteration) — **not safe** for concurrent component processing as
  written (concurrent `push_back` on the same vector from multiple threads is a data race), and
  additionally the **order** components are appended in directly determines
  `results_summary`/`errors` order in the final YAML (`SimulationOutputWriter.cpp`'s
  `groupBySameResult` uses `std::stable_sort`, so ties are broken by insertion order; the
  competitive-mode `writeCompetitiveReport` re-sorts by score/steps but also `std::stable_sort`s,
  same insertion-order tie-break; `errors_node`/`failures` are emitted in whatever order the input
  vector holds, no sort at all). This is the subject of Stage 2 below (fixed via preallocated,
  per-index result slots — no lock needed for this part at all).
- **`Simulator/src/SimulationRunFactoryImpl.cpp`, `SimulationRunImpl.cpp`, `Map3DImpl.cpp`,
  `MockGPS.cpp`, `MockLidar.cpp`, `MockMovement.cpp`, `MapsComparison.cpp`, `ConfigLoader.cpp`** —
  grepped for `static`/`thread_local`/`extern` mutable state (excluding `static_cast`/
  `static_assert`); found only `Registrar`'s own singleton (already covered above) and one
  read-only `static constexpr std::array` in `Algorithm/src/MappingAlgorithmImpl.cpp`. Every
  object these files construct (hidden map, output map, GPS/lidar/movement mocks, the
  `SimulationRunFactoryImpl`/`SimulationManager` pair itself) is created fresh per component, owned
  by `unique_ptr`s local to that component's processing — **safe for one component per thread,
  no cross-component sharing of any of these objects.**
- **The composition data itself** (`ParsedComposition` — `types::SimulationCompositionData
  composition` + `CompositionFilePaths file_paths`, from `Simulator/include/Simulator/
  ConfigLoader.h`) is parsed exactly **once**, before the per-component loop, and passed by
  `const&` into every component's `SimulationManager::run(...)` call already, both today and
  after this plan. It is plain aggregate data (strings/vectors/structs), never mutated after
  `parseCompositionData` returns — **safe to read concurrently from every worker thread.**
- **The "fixed" component's factory** (`options.algorithm_so_file`'s
  `common::MappingAlgorithmFactory` in comparative mode; `options.mission_control_so_file`'s
  `common::MissionControlFactory` in competition mode) is loaded once, then its `std::function`
  is invoked **concurrently by every worker thread** (one call per component, each producing an
  independent instance) once this plan's Stage 2 lands. This is safe here because: (a) the
  `REGISTER_MAPPING_ALGORITHM`/`REGISTER_MISSION_CONTROL` macros (`common/include/Common/
  MappingAlgorithmRegistration.h` / `MissionControlRegistration.h`) generate a captureless lambda
  (`[](Dependencies d) -> ... { return std::make_unique<class_name>(std::move(d)); }`), so
  invoking the same `std::function` concurrently is just concurrent read-only dispatch, and
  (b) `Algorithm/src/MappingAlgorithmImpl.cpp` and `MissionControl/src/MissionControlImpl.cpp` /
  `DroneControlImpl.cpp` (this repo's own implementations) were grepped and contain no shared
  mutable state beyond the one read-only array already noted. **Do not weaken this precondition**:
  if either implementation file changes in the future to add any global/static mutable state, or
  a cache, that would silently reintroduce a race here — worth a one-line note in `README.md`
  (see Stage 3).

**Two things this audit could not fully verify by header inspection alone — spot-check them as
the first thing you do in Stage 1, before writing any other code:**
1. `TinyNPY.h`'s `NpyArray::LoadNPY` (used by `Simulator/src/SimulationRunFactoryImpl.cpp`'s
   `loadHiddenMap` — every component, every run, loads its own hidden map file). Read the
   vendored source (after the devcontainer's vcpkg install, under
   `build/default/vcpkg_installed/x64-linux/include/TinyNPY.h`, and its `.cpp`/inline
   implementation if the header doesn't contain everything) and confirm `LoadNPY` doesn't rely on
   any static/global mutable buffer, a shared error-message pointer, or unguarded shared state.
   Different components load different map files concurrently once Stage 2 lands.
2. `yaml-cpp`'s `YAML::Node`/`YAML::LoadFile`/emitter (used by `ConfigLoader.cpp` once,
   sequentially before threading — not a concern — but also by `SimulationOutputWriter.cpp`'s
   `writeSimulationOutput`, called once per component, potentially concurrently after Stage 2).
   Confirm each call builds and emits its own independent `YAML::Node` tree with no shared global
   emitter state (this is yaml-cpp's documented/expected usage pattern, but confirm by reading
   the vendored `include/yaml-cpp` headers rather than assuming).

If either check turns up a real shared-mutable-state hazard, stop and either (a) serialize just
that call behind a dedicated `std::mutex` (cheap: it's a per-component, not per-run, call site),
or (b) if it looks like a deeper library limitation, write down the exact finding and raise it
before proceeding — don't silently work around a real bug in a way that could reintroduce the
same race elsewhere.

---

## Stage 1 — Thread-safe `std::cerr` context logging

**Problem being fixed:** `Simulator/include/Simulator/CerrContextGuard.h` /
`Simulator/src/CerrContextGuard.cpp` implement per-run/per-component error-log line prefixing
(`"[component=X] [sim=... mission=... drone=... lidar=...] "`) by swapping `std::cerr.rdbuf()` to
a new `PrefixingStreambuf` on construction and restoring the previous one on destruction, nested
arbitrarily deep. `std::cerr` is one process-wide object with exactly one mutable `rdbuf()`
pointer; two threads each constructing/destroying a `CerrContextGuard` concurrently will
interleave those swaps unpredictably (a guard destructing on thread A can restore a buffer thread
B is still supposed to be nested inside, or two threads can race to install a buffer, corrupting
which "current" buffer either thread's writes land in). Separately — and this holds even if
`rdbuf()` swapping were somehow made safe — two threads' `std::cerr << ...` statements are each a
*sequence* of several chained `operator<<` calls; even though the C++ standard guarantees
`std::cerr` itself won't suffer a data race across threads (27.4.1: concurrent use of `cin`/
`cout`/`cerr`/`clog` is safe against data races when `sync_with_stdio` is true, the default, which
nothing in this codebase disables), it explicitly leaves the **interleaving of characters output
by multiple threads unspecified** — so without extra care, two components' log lines could still
interleave mid-line into `error.log`, which would defeat the entire point of the context prefixes.

**Change:** `Simulator/include/Simulator/CerrContextGuard.h`, `Simulator/src/CerrContextGuard.cpp`
(full rewrite of internals; the two public class names stay, `CerrContextGuard`'s public
constructor signature stays `explicit CerrContextGuard(const std::string& context_label)`, so
every existing call site — `SimulationManager.cpp`'s `runInternal`, and the (about to be rewritten
in Stage 2) `SimulatorRunner.cpp` — needs **zero changes**).

**New design:**

1. Add a new class `CerrSinkGuard` to the header:
   ```cpp
   // Installs the single, process-wide, thread-safe std::cerr sink for its lifetime, restoring
   // std::cerr's previous streambuf on destruction. Construct exactly once, on whichever thread
   // owns std::cerr's lifetime for this run (main(), or a test's setup code), strictly before any
   // other thread that might write to std::cerr is started, and keep it alive until every such
   // thread has been joined. CerrContextGuard's per-thread context labels (below) only take
   // effect while a CerrSinkGuard is alive somewhere on the call stack; without one, std::cerr
   // behaves exactly as if this whole mechanism didn't exist (plain, unprefixed passthrough is
   // NOT provided — a CerrSinkGuard must always be installed before this codebase writes to
   // std::cerr for prefixes to appear; see main()'s existing error_log setup in
   // drone_mapper_simulation_main.cpp, and note below for how existing tests must adapt).
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
   ```
2. `CerrContextGuard` keeps its existing public shape but its implementation becomes a pure
   thread-local label stack push/pop — it no longer touches `std::cerr` or any streambuf at all:
   ```cpp
   class CerrContextGuard {
   public:
       explicit CerrContextGuard(const std::string& context_label);
       ~CerrContextGuard();
       CerrContextGuard(const CerrContextGuard&) = delete;
       CerrContextGuard& operator=(const CerrContextGuard&) = delete;

       // Returns "[label1] [label2] ... " for every guard currently alive on the CALLING
       // thread's stack, outermost first, or "" if none. Used internally by
       // CerrSinkGuard::ConcurrentContextStreambuf; exposed so it's independently unit-testable.
       [[nodiscard]] static std::string currentPrefix();
   };
   ```
3. In the `.cpp`, a function-local `thread_local std::vector<std::string>` holds each thread's
   own stack of active labels; the constructor `push_back`s, the destructor `pop_back`s,
   `currentPrefix()` joins them as `"[" + label + "] "` concatenated in stack order (outermost
   first — matches the exact existing nested-prefix text format, e.g.
   `"[component=aaa_mgr.so] [sim=... mission=... drone=... lidar=...] "`, so no consumer of
   `error.log`'s text format sees any difference). Being `thread_local`, two threads' stacks are
   completely independent — no synchronization needed for this part at all.
4. `ConcurrentContextStreambuf` (nested inside `CerrSinkGuard`, defined only in the `.cpp`) wraps
   one real destination `std::streambuf* dest_` and one `std::mutex write_mutex_`. Its
   `overflow(int ch)`:
   - **First, handle `traits_type::eof()`, returning the success sentinel, not `eof()` itself:**
     `if (ch == traits_type::eof()) { return traits_type::not_eof(ch); }` before touching anything
     else. Per `std::streambuf::overflow`'s documented contract, the return value is `eof()` to
     signal *failure* and anything else to signal success — returning `ch` unmodified in this
     branch would literally return `eof()`, which formatted-output call sites (`operator<<`)
     interpret as a failure and can set the stream's `failbit`, silently breaking all further
     `std::cerr` output for the rest of the process. `traits_type::not_eof(ch)` is the standard
     idiom for "nothing to write, no error" here. (Note: `PrefixingStreambuf`, the class this
     replaces, returns `ch` unmodified in this branch — do not copy that; verify against the
     contract above, not against the prior implementation.)
   - Otherwise, append the character to a **function-local `thread_local std::string`** (i.e. one
     accumulation buffer per thread — never shared, so no synchronization needed for the
     accumulation step itself, only for the final flush below).
   - When the appended character is `'\n'`: compute `CerrContextGuard::currentPrefix()`, then
     `{ std::lock_guard<std::mutex> lock(write_mutex_); dest_->sputn(prefix...); dest_->sputn(line...); }`,
     then clear the thread-local buffer. This is the **only** lock this plan introduces beyond
     `Registrar`'s pre-existing one, and its critical section is short (one line's worth of
     bytes) — consistent with "prefer no locking, lock where genuinely needed." Deliberately do
     **not** call `dest_->pubsync()`/flush here: the mutex is what guarantees correct,
     non-interleaved ordering of these `sputn` calls, independent of whatever internal buffering
     `dest_` performs — "written" in this design means "queued through `dest_` under
     `write_mutex_`," not "synced to persistent storage immediately," and nothing in this plan
     requires per-line disk durability. For the real destination in `main()` (`error_log`'s
     filebuf), final on-disk persistence is guaranteed by `error_log`'s own destructor, which runs
     after `CerrSinkGuard`'s (see the declaration-order note below); for `CerrCapture`'s
     `std::ostringstream` destination in tests, there is no separate flush step at all — `sputn`
     is immediately visible via `buffer_.str()`.
   - **Required precondition, already true today (verified by grep):** every `std::cerr << ...`
     call site in this codebase ends its statement with a literal `'\n'` (none use `std::endl`,
     none leave a trailing partial line). This is not just a passive observation — it's what
     guarantees every thread's accumulation buffer is empty whenever no write is in progress (see
     point 6 below for why that specifically matters). A message that never sends a `'\n'` would
     never flush under this design; if a future call site needs one, it must send its own trailing
     `'\n'` — this design deliberately adds no destructor-time or size-based fallback flush.
   - Do **not** try to rely on any assumption that the C++ standard's `cin`/`cout`/`cerr`/`clog`
     data-race guarantee also serializes calls into a *custom* installed streambuf's `overflow` —
     it doesn't need to for this design to be correct, because the thread-local accumulation
     buffer is exclusively owned by its thread regardless of how `overflow` gets invoked; the
     explicit mutex is what actually protects the shared `dest_`.
5. `CerrSinkGuard`'s constructor: `buf_ = std::make_unique<ConcurrentContextStreambuf>(destination);
   original_ = std::cerr.rdbuf(buf_.get());`. Destructor: `std::cerr.rdbuf(original_);` (matches
   the existing `CerrRedirectGuard` pattern in `drone_mapper_simulation_main.cpp` — see next
   point).
6. **At most one `CerrSinkGuard` may be alive at any moment, process-wide — never nested, never
   overlapping across threads.** This is a hard precondition, not a suggestion: the thread-local
   accumulation buffer in `overflow` (point 4) is scoped to the thread, not to one
   `ConcurrentContextStreambuf` instance, so two overlapping instances could flush a thread's
   buffered text through the wrong one. It's safe to construct further `CerrSinkGuard`s later in
   the same process **sequentially** (one per `main()` invocation, or one per `TEST()` case in
   `simulator_registration_test`) precisely because point 4's "always ends with `'\n'`"
   precondition, plus this class's own contract of never being destroyed before every writer
   thread is joined, together guarantee every thread's buffer is empty whenever no `CerrSinkGuard`
   is active — no reset logic is needed between instances. Enforce "at most one, ever" with a
   runtime check that stays active in **every** build configuration — not `assert()`, which
   `NDEBUG` compiles out entirely, and this project's actual build/grading configuration is not
   guaranteed to leave assertions enabled: a process-wide `static std::atomic<bool> g_active` in
   the `.cpp`, checked via `exchange(true)` as the very first thing in the constructor body
   (before touching `std::cerr` at all), and if it was already `true`,
   `throw std::logic_error("CerrSinkGuard: only one instance may be active at a time")` — matching
   this codebase's existing precedent for constructor-time invariant violations
   (`SimulationRunImpl`'s constructor already throws `std::invalid_argument` for an analogous
   "this must never happen" case). Clear the flag with `g_active.store(false)` in the destructor.

   **Roll `g_active` back to `false` if anything after the successful `exchange(true)` throws
   before construction completes.** `std::make_unique<ConcurrentContextStreambuf>(destination)`
   (point 5) allocates and can throw `std::bad_alloc`; if it (or anything else in the constructor
   body) throws after `g_active` was already claimed, `CerrSinkGuard`'s constructor never
   completes, so `~CerrSinkGuard()` is never invoked for it — a partially-constructed object's
   destructor does not run — and `g_active` would stay `true` permanently, making every later,
   perfectly legitimate `CerrSinkGuard` construction for the rest of the process incorrectly throw
   the "already active" error. Wrap everything after the claim in a `try`/`catch (...)` that resets
   the flag and rethrows:
   ```cpp
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
   ```
   This also means: keep running this binary's tests the way it already does today — gtest's
   default sequential-within-one-process execution — not under a runner that executes `TEST()`
   cases as concurrent threads sharing one process.

**Also touched:** `Simulator/src/drone_mapper_simulation_main.cpp` — delete the local
`CerrRedirectGuard` class (lines ~14–28 today) entirely; it's superseded by
`simulator::CerrSinkGuard`. Change `const CerrRedirectGuard cerr_guard(error_log.rdbuf());` to
`const simulator::CerrSinkGuard cerr_guard(error_log.rdbuf());` (add `#include
<Simulator/CerrContextGuard.h>` if not already present via another header). No other change to
`main()`'s logic, ordering, or error handling around opening `error_log`. This ordering is load-bearing
and must not be disturbed: `error_log` is already declared, and therefore constructed, before
`cerr_guard` — local variables destroy in reverse declaration order, so `cerr_guard` is destroyed
(restoring `std::cerr`) before `error_log` closes, keeping the destination alive for the guard's
entire lifetime. Do not reorder these two declarations.

**Also touched:** `Simulator/tests/stage3_verify/stage3_verify_test.cpp`'s `CerrCapture` helper
(lines ~53–63 today) currently does raw `std::cerr.rdbuf(buffer_.rdbuf())` — a plain
`std::ostringstream`'s buffer, no prefixing logic of its own; prefixing worked automatically only
because the *old* `CerrContextGuard` itself installed prefixing buffers on top of whatever was
already there. Under the new design, `CerrContextGuard` no longer touches `rdbuf()` at all, so
`CerrCapture` must install a `simulator::CerrSinkGuard` wrapping its buffer instead:
```cpp
class CerrCapture {
public:
    CerrCapture() : sink_(buffer_.rdbuf()) {}
    [[nodiscard]] std::string str() const { return buffer_.str(); }
private:
    std::ostringstream buffer_;
    simulator::CerrSinkGuard sink_; // must be constructed after buffer_ (member init order below)
};
```
The member declaration order shown above is required, not incidental: `buffer_` must be declared
**before** `sink_` in the class body, because member initialization always runs in declaration
order regardless of the order members are listed in the constructor's member-initializer list —
`sink_(buffer_.rdbuf())` is only well-defined if `buffer_` has already been constructed by the time
`sink_`'s constructor runs. The same order guarantees correct teardown too: members destroy in
reverse of declaration order, so `sink_` is destroyed before `buffer_` — `buffer_` therefore
remains alive for `sink_`'s entire lifetime, exactly what `CerrSinkGuard`'s contract requires of
its destination. Keep exactly the declaration order shown (`buffer_` first, `sink_`
second); do not declare `sink_` first, and do not replace this with a two-step constructor body
that default-constructs `sink_` and assigns into it afterward — `CerrSinkGuard` is
non-default-constructible and non-copyable/non-movable (RAII-only, matching `CerrContextGuard`'s
existing style), so there is no "assign after the fact" option available here; the
member-declaration-order approach above is the only correct way to wire this up. This is the only
edit needed in that test file for Stage 1 — its four existing `TEST(Stage3Verify, ...)` cases keep
working unchanged once this helper is fixed, since they only ever call `capture.str()` and check
substrings.

**New verification (add here, don't wait for Stage 3):** add a small, focused test: a new file
`Simulator/tests/cerr_context_verify/cerr_context_verify_test.cpp`, folded into
`simulator_registration_test` (mirroring the existing stage-N-verify convention — add this one
`.cpp` file to `Simulator/CMakeLists.txt`'s `simulator_registration_test` source list; no new
sources are needed beyond what that target already links, since this test only needs
`CerrContextGuard.cpp`, already a dependency of the target), covering:
1. **Single-thread nesting still matches the old text format exactly:** install a
   `CerrSinkGuard` over a captured buffer, construct nested `CerrContextGuard`s (`"component=X"`
   then `"sim=Y"`), write a line, confirm the captured text is exactly
   `"[component=X] [sim=Y] hello\n"`.
2. **No prefix outside any guard:** write a line with no `CerrContextGuard` alive, confirm it's
   unprefixed (matches today's behavior for e.g. `main()`'s top-level catch-block message).
3. **Concurrent, non-interleaving, correctly-attributed lines:** install one `CerrSinkGuard`, then
   spawn e.g. 8 `std::thread`s, each looping ~500 times: construct a `CerrContextGuard` with a
   thread-unique label (e.g. `"thread=" + std::to_string(i)`), write one line containing that same
   index, destroy the guard, repeat. Join all threads. Parse every line in the captured buffer and
   assert: (a) every line matches `^\[thread=(\d+)\] iteration \d+$` (or similar) — i.e. no line is
   missing its bracket, truncated, or has two different threads' text concatenated on one line —
   and (b) the thread index inside the brackets matches the index embedded in that same line's
   body (proves the prefix and the body were never computed/attributed from different threads'
   concurrent activity). Do **not** assert anything about which thread's lines appear before which
   other thread's lines in the captured buffer — only completeness (every thread's every iteration
   produced exactly one line), non-interleaving, and attribution, per the `error.log` ordering
   clarification in "Hard requirements" above. Run this test under both a normal build and, if the
   devcontainer has it available, a `-fsanitize=thread` build (see Stage 3) to catch anything a
   single run's scheduling happened not to expose.
4. **Single-active-guard invariant is enforced, not just documented:** construct one
   `CerrSinkGuard`, then, while it is still alive, confirm that constructing a second one over a
   different scratch buffer throws — e.g. `EXPECT_THROW(simulator::CerrSinkGuard second(other.rdbuf()),
   std::logic_error)` — rather than silently running with two overlapping instances. Prefer this
   throw-based check over a death test (`EXPECT_DEATH`): it's portable, and because it doesn't rely
   on `assert`, it verifies the real behavior regardless of whether the test binary happens to be
   built with `NDEBUG`.

**Must not change:** `SimulationManager.cpp`'s call site (`const CerrContextGuard cerr_guard(contextLabel(...))`)
— its signature and behavior from the caller's point of view are unchanged. `Registrar.*`,
`SimulationOutputWriter.*`, `ConfigLoader.*`, anything under `common/`/`Algorithm/`/
`MissionControl/`.

---

## Stage 2 — Concurrent component execution in `SimulatorRunner.cpp`

**Precondition:** Stage 1 complete and its new test passing — this stage's correctness depends on
`CerrContextGuard` already being safe to construct/destroy concurrently from multiple threads.

**Change:** `Simulator/src/SimulatorRunner.cpp`, plus one small addition to
`Simulator/include/Simulator/SimulatorRunner.h` (the `computeWorkerCount` declaration required by
this stage's Verify section below — see there for the exact snippet). `runComparative`'s and
`runCompetition`'s existing public signatures in that header are unchanged. No CMake changes
(`<thread>`, `<atomic>`, `<mutex>` need no new `find_package`/`target_link_libraries` —
`Threads::Threads` is already linked in `Simulator/CMakeLists.txt` for both the main executable and
`simulator_registration_test`).

**Required decisions (all already settled by the analysis above — implement exactly this):**

1. **Worker-count formula**, as a small free function in the file's anonymous namespace (exposed
   non-anonymously enough to unit-test — see Verify below):
   ```cpp
   // work_items = the number of independent components actually queued for this call (i.e.
   // mission_control_libraries.size() in comparative mode, algorithm_libraries.size() in
   // competition mode — always >= 1, guaranteed by CliOptions' folder validation).
   std::size_t computeWorkerCount(const std::optional<int>& num_threads, std::size_t work_items) {
       if (!num_threads.has_value() || *num_threads <= 1 || work_items <= 1) {
           return 0; // run sequentially on the calling (main) thread
       }
       return std::min<std::size_t>(static_cast<std::size_t>(*num_threads), work_items);
   }
   ```
   This can never return `1`: the `work_items <= 1` branch catches the only way a naive
   `min(...)` could yield `1`, and otherwise both operands of `min` are `>= 2`. Total thread count
   is therefore always exactly `1` (worker_count == 0) or `>= 3` (worker_count + 1, worker_count
   >= 2) — never `2`, satisfying the assignment's stated invariant by construction, not by luck.

2. **Work distribution — a shared atomic cursor, not static partitioning:** components can take
   wildly different amounts of wall-clock time (a big composition sweep vs. a small one, a
   `.so` that fails to load immediately vs. one that runs the full sweep), so a fixed
   "thread *t* gets items `[t*k, (t+1)*k)`" split risks leaving some threads idle while one is
   still grinding through a slow component. Use one `std::atomic<std::size_t>` counter that every
   worker thread (and, in the `worker_count == 0` case, the single sequential loop) pulls the next
   index from via `fetch_add`; no other synchronization is needed for distributing the work
   itself:
   ```cpp
   // Runs process(i) for every i in [0, item_count). worker_count == 0 means "run sequentially,
   // in order, on the calling thread" — used for num_threads absent/1, or when there's only one
   // component to process, so this call is a strict superset of (and, for worker_count == 0,
   // textually identical in control flow to) the pre-multithreading sequential loop. Otherwise
   // spawns worker_count std::threads that each repeatedly claim the next unclaimed index via a
   // shared atomic cursor until none remain, then joins all of them before returning — the
   // calling thread does not itself call process() in this branch (matches the assignment's "the
   // main thread waiting in a join for all workers is fine" framing: once threading is engaged,
   // main is purely a dispatcher/joiner).
   template <typename F>
   void runIndexed(std::size_t item_count, std::size_t worker_count, F&& process) {
       if (worker_count == 0) {
           for (std::size_t i = 0; i < item_count; ++i) {
               process(i);
           }
           return;
       }
       std::atomic<std::size_t> next{0};
       std::vector<std::thread> workers;
       workers.reserve(worker_count);
       try {
           for (std::size_t t = 0; t < worker_count; ++t) {
               workers.emplace_back([&next, item_count, &process]() {
                   for (;;) {
                       const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
                       if (i >= item_count) {
                           break;
                       }
                       try {
                           process(i);
                       } catch (...) {
                           // process(i) is expected to catch and record every failure mode it
                           // knows about itself (point 4's own two try/catch blocks); reaching
                           // here means something escaped both of those (e.g. std::bad_alloc from
                           // bookkeeping around them, such as building component_name or
                           // constructing CerrContextGuard). Deliberately swallow unconditionally,
                           // with NO recovery code of any kind, not even logging: any recovery
                           // attempt here is itself potentially-throwing (string/path operations,
                           // std::cerr writes going through Stage 1's thread-local buffer), which
                           // would defeat the entire point of this catch -- an empty body is the
                           // only way to guarantee it cannot itself throw. This lives here, in
                           // runIndexed's threaded branch, and deliberately NOT inside process(i)
                           // itself: process(i) is called from both branches of runIndexed, and
                           // this plan's "must preserve byte-for-byte sequential behavior"
                           // requirement means the sequential (worker_count == 0) branch below
                           // must keep propagating such an exception exactly as the
                           // pre-multithreading code always did -- all the way to main()'s
                           // top-level catch, aborting the whole run -- not silently downgrade it
                           // to a per-component failure. A component lost this way is left at
                           // outcomes[index]'s default (empty component_name, no totals) and still
                           // lands in errors: via the final pass below, just without a name or a
                           // log line -- an accepted, deliberately minimal degradation for a
                           // failure mode this narrow.
                       }
                   }
               });
           }
       } catch (...) {
           // A std::thread constructor above threw -- std::thread's documented failure mode is
           // std::system_error when the OS refuses to start another thread, a real possibility
           // under resource pressure (e.g. a large num_threads on a loaded system), not just
           // theoretical. Every worker started so far is still joinable; `workers`' own destructor,
           // about to run as this function unwinds, calls std::terminate() on ANY std::thread it
           // destroys while still joinable -- so join every already-started worker right here,
           // before letting the exception continue propagating, or the process crashes regardless
           // of any catch clause elsewhere in the call chain.
           for (std::thread& w : workers) {
               w.join();
           }
           throw;
       }
       for (std::thread& w : workers) {
           w.join();
       }
   }
   ```
   No exception may ever escape a worker thread's call to `process(i)` — an uncaught exception
   inside a `std::thread`'s function calls `std::terminate()` on the whole process, which would
   violate "the Simulator shall not crash" for anything short of an actual MissionControl/Algorithm
   crash. `process`'s own two application-level try/catch blocks (point 4) cover exceptions from
   `Registrar`/`SimulationManager`/etc., but something can still throw in the bookkeeping *around*
   them. The catch that closes that gap lives **inside `runIndexed`'s threaded branch above**
   (wrapping each `process(i)` call), not inside `process` itself — see the code comment above for
   why: `process` is called from both branches of `runIndexed`, and only the threaded branch may
   change what happens when something escapes both of its own try/catch blocks; the sequential
   branch must not.

3. **Preallocated, per-index result slots — no lock needed for results either:**
   ```cpp
   struct ComponentOutcome {
       std::string component_name;                // set as the very first statement in the
                                                    // per-index lambda (point 4), before anything
                                                    // else that could throw -- stays at its
                                                    // default (empty string) only in the one
                                                    // irreducible case where computing it
                                                    // (library_path.filename().string() itself)
                                                    // throws before any assignment can happen; see
                                                    // point 4's ordering and point 2's runIndexed
                                                    // safety net. Not a case this design can fully
                                                    // close (identifying the component requires
                                                    // computing its name first), only narrow to
                                                    // that one unavoidable operation.
       std::optional<ComponentRunTotals> totals;   // set only if this component ran successfully
   };
   ```
   Allocate `std::vector<ComponentOutcome> outcomes(work_items)` **before** calling `runIndexed`.
   Each call to `process(i)` writes **only** to `outcomes[i]` — since indices are claimed
   exclusively (each `i` is handed to exactly one `fetch_add` caller, or produced by exactly one
   sequential-loop iteration), no two threads ever touch the same element, so this requires no
   lock at all (distinct vector elements are not a data race per the C++ memory model, as long as
   the vector itself is never resized while `runIndexed` is in flight — it isn't; it's sized once,
   up front). After `runIndexed` returns (all workers joined, or the sequential loop finished),
   walk `outcomes` **in order** (single-threaded, back on the calling thread) to rebuild the exact
   two vectors the existing report writers expect, preserving today's ordering guarantees:
   ```cpp
   std::vector<ComponentRunTotals> totals;
   std::vector<std::string> failures;
   for (ComponentOutcome& outcome : outcomes) {
       if (outcome.totals) {
           totals.push_back(std::move(*outcome.totals));
       } else {
           failures.push_back(outcome.component_name);
       }
   }
   ```
   Because `outcomes` is indexed by each component's position in the already-`sortedByFilename`
   list (sorting is unchanged, still happens before this loop), this reproduces **exactly** the
   same `totals`/`failures` order the current sequential `push_back`-in-loop-order code produces,
   regardless of which thread finished which component first or how long each took — this is what
   makes `results_summary`/`errors` ordering deterministic and reproducible under concurrency, not
   just under sequential execution.

4. **Per-component body — move the existing loop body into `process(index)` almost verbatim, and
   keep `runOneComponent` as a shared helper with an updated signature.** `runOneComponent` (the
   free helper already shared by both modes today) currently takes
   `std::vector<ComponentRunTotals>& totals, std::vector<std::string>& failures` by reference and
   `push_back`s its result into one of them; that specific shape can't survive this stage (no two
   threads may share a mutable accumulation target), but the run/write-phase logic it wraps —
   constructing `SimulationRunFactoryImpl`, running a fresh `SimulationManager`, writing this
   component's YAML, computing its totals, and the exception message on failure — is identical
   between comparative and competitive mode and is genuinely non-trivial. Duplicating that body at
   two call sites instead of updating one shared helper's signature would leave two copies of the
   same try/catch logic and exception message text that a future change would have to remember to
   keep in sync — real, avoidable maintenance risk for no benefit. **Update its signature to take a
   `ComponentOutcome&` for its one index instead of the two shared vectors; keep everything else
   about it unchanged:**
   ```cpp
   // Run/write phase for one already-loaded component: builds a fresh SimulationRunFactoryImpl
   // from the two given factories, runs the whole composition through a fresh SimulationManager,
   // writes this component's own detailed YAML, and records its aggregate totals into
   // outcome.totals -- or, on any exception from that whole phase, logs the reason and leaves
   // outcome.totals unset (the caller then records this component as failed via
   // outcome.component_name, already set by the caller before this is invoked). Shared between
   // runComparative's and runCompetition's per-index lambda (point 4), which differ only in which
   // side is "fixed" vs. "looped" -- this helper doesn't need to know that.
   void runOneComponent(const std::string& component_name, const std::string& component_stem,
                        const ParsedComposition& parsed, const std::filesystem::path& results_dir,
                        const common::MappingAlgorithmFactory& mapping_algorithm_factory,
                        const common::MissionControlFactory& mission_control_factory, bool verbose,
                        ComponentOutcome& outcome) {
       try {
           auto factory_impl = std::make_unique<SimulationRunFactoryImpl>(
               mapping_algorithm_factory, mission_control_factory, verbose);
           SimulationManager manager{std::move(factory_impl)};
           const types::SimulationManagerReport report =
               manager.run(parsed.composition, results_dir / component_stem, parsed.file_paths);
           writeSimulationOutput(report, parsed.file_paths,
                                 results_dir / ("simulation_output_" + component_stem + ".yaml"));
           outcome.totals = computeComponentTotals(component_name, report);
       } catch (const std::exception& e) {
           std::cerr << "component " << component_name << " failed during simulation/output: " << e.what() << '\n';
       }
   }
   ```
   Rewrite `runComparative` as (symmetric rewrite for `runCompetition`, swapping which factory
   type is fixed vs. looped, exactly as today):
   ```cpp
   std::size_t runComparative(const ComparativeOptions& options, const std::filesystem::path& results_dir) {
       const ParsedComposition parsed = parseCompositionData(options.simulation_composition_file);

       // Fixed component: unchanged from today — loaded once, before any worker thread exists,
       // on the calling thread; a load failure here still propagates straight out, uncaught, to
       // main()'s top-level catch (see SIMULATOR_CORE_PLAN.md Stage 3's original reasoning, which
       // still applies unchanged: nothing meaningful can run without it, and neither YAML schema
       // has an errors:-list slot for it).
       const common::MappingAlgorithmFactory mapping_algorithm_factory =
           Registrar::instance().loadMappingAlgorithm(options.algorithm_so_file).front();

       const std::vector<std::filesystem::path> mission_control_libraries =
           sortedByFilename(options.mission_control_libraries);

       std::vector<ComponentOutcome> outcomes(mission_control_libraries.size());
       const std::size_t worker_count =
           computeWorkerCount(options.num_threads, mission_control_libraries.size());

       runIndexed(mission_control_libraries.size(), worker_count, [&](std::size_t index) {
           const std::filesystem::path& library_path = mission_control_libraries[index];
           // Set outcomes[index].component_name as the very first statement, before anything
           // else in this lambda that could throw (component_stem, CerrContextGuard's
           // constructor, ...) -- narrows the window in which the runIndexed-level safety net
           // (point 2) could leave this slot at its default empty name to just this one,
           // unavoidable operation.
           outcomes[index].component_name = library_path.filename().string();
           const std::string& component_name = outcomes[index].component_name;
           const std::string component_stem = library_path.stem().string();

           const CerrContextGuard component_guard("component=" + component_name);

           std::vector<common::MissionControlFactory> mission_control_factories;
           try {
               mission_control_factories = Registrar::instance().loadMissionControl(library_path);
           } catch (const std::exception& e) {
               std::cerr << "failed to load " << component_name << ": " << e.what() << '\n';
               return; // outcomes[index].totals stays nullopt -> recorded as a failure below
           }

           runOneComponent(component_name, component_stem, parsed, results_dir, mapping_algorithm_factory,
                           mission_control_factories.front(), options.verbose, outcomes[index]);
       });

       std::vector<ComponentRunTotals> totals;
       std::vector<std::string> failures;
       for (ComponentOutcome& outcome : outcomes) {
           if (outcome.totals) {
               totals.push_back(std::move(*outcome.totals));
           } else {
               failures.push_back(outcome.component_name);
           }
       }

       writeComparativeReport(options.simulation_composition_file, options.mission_control_folder,
                             totals, failures, results_dir / "comparative_report.yaml");
       return totals.size();
   }
   ```
   Every message, exception type caught, and log-string format inside the two try/catch blocks and
   `runOneComponent` itself is copied verbatim from the existing code, and this lambda has no
   wrapping of its own beyond what the existing code already had — this is a mechanical relocation
   of the existing per-iteration body into a lambda plus a mechanical change of "append to a shared
   vector" into "write to my own `outcomes[index]`," not a rewrite of any existing logic. (The
   worker-thread-only safety net for exceptions that escape both of these try/catch blocks lives in
   `runIndexed`, point 2 above — deliberately not here; see that point's reasoning.)

5. **`results_dir / component_stem` subdirectory creation still needs no synchronization**:
   `SimulationManager::runInternal`'s `std::filesystem::create_directories(leaf_dir)` (unchanged
   by this plan) only ever creates paths nested under `results_dir / component_stem`, and every
   component's `component_stem` is distinct (guaranteed: folder entries have unique filenames, and
   `CliOptions.cpp`'s folder scan only admits entries whose extension is exactly `.so`, so two
   distinct filenames can't collide to the same stem). `results_dir` itself already exists before
   any of this runs (created once by `main()`, single-threaded, before `runComparative`/
   `runCompetition` is even called). Two threads creating two different, non-overlapping
   subdirectories under one already-existing parent directory concurrently is an ordinary,
   race-free filesystem operation — no lock needed here, and this plan does not add one.

**Verify:**
- **Expose `computeWorkerCount` for direct unit testing with exactly this approach — no other
  approach is acceptable:** move its definition out of `SimulatorRunner.cpp`'s anonymous namespace
  (plain `simulator::` external linkage, not `static`), and add its declaration to
  `Simulator/include/Simulator/SimulatorRunner.h`, directly below the existing
  `runComparative`/`runCompetition` declarations, under a comment marking it as an internal helper
  kept in the public header only so tests can reach it — e.g.:
  ```cpp
  // Internal helper, declared here only so tests can call it directly -- not part of the
  // orchestration API above; runComparative/runCompetition remain the only entry points anything
  // outside this file and its tests should use. work_items is the number of components queued for
  // this call (mission_control_libraries.size() in comparative mode, algorithm_libraries.size()
  // in competition mode); see SimulatorRunner.cpp for the exact rule this implements.
  [[nodiscard]] std::size_t computeWorkerCount(const std::optional<int>& num_threads,
                                               std::size_t work_items);
  ```
  Do **not** create a separate test-only header for this, and do **not** try to forward-declare it
  from inside the anonymous namespace from the test file instead — anonymous-namespace symbols
  have internal linkage, so a separate translation unit (the test `.cpp`) cannot link against one
  at all, forward-declared or not; leaving it in the anonymous namespace and declaring it again
  from the test file would compile but fail to link. The header-declaration approach above is the
  only one that both works and keeps the change minimal (no new file, no new CMake target). Add
  `TEST(SimulatorRunnerVerify, ComputeWorkerCount)` covering at minimum:
  `computeWorkerCount(std::nullopt, 5) == 0`; `computeWorkerCount(1, 5) == 0`;
  `computeWorkerCount(5, 1) == 0` (**the critical never-total-2 case**);
  `computeWorkerCount(4, 1) == 0`; `computeWorkerCount(4, 10) == 4`;
  `computeWorkerCount(20, 10) == 10`.
- Re-run every existing `Stage3Verify` test in `stage3_verify_test.cpp` completely unchanged
  (besides the `CerrCapture` fix from Stage 1) and confirm they still pass — these all call
  `runComparative`/`runCompetition` with `num_threads` left unset (`std::nullopt`, since
  `parseComparative`/`parseCompetition`'s test helper never passes `num_threads=`), so they
  exercise the `worker_count == 0` sequential path and must produce byte-identical results to
  before this stage.
- Add a new test file (fold into `simulator_registration_test`, matching the existing
  stage-N-verify convention — e.g. `Simulator/tests/multithreading_verify/
  multithreading_verify_test.cpp`, add it to `Simulator/CMakeLists.txt`'s
  `simulator_registration_test` source list, no new sources needed beyond what that target already
  links).

  **Registrar's one-load-per-literal-path-per-process constraint applies to every test below,
  including comparisons across multiple invocations within the same test — and applies to *every*
  `.so` path each call loads, not only the looped ones:** `Registrar::loadMissionControl`/
  `loadMappingAlgorithm` only freshly `dlopen`s a path the first time that literal path is loaded
  in this process — a second `dlopen` of the *same* path does not rerun its `REGISTER_*`
  constructor, so a repeated load of the same path always fails with `PLUGIN_NOT_REGISTERED`,
  regardless of whether the first load succeeded (already documented, and already worked around
  this way, in `stage3_verify_test.cpp`'s own comments). Each `runComparative`/`runCompetition`
  call loads **two kinds** of `.so` path: the looped components
  (`mission_control_libraries`/`algorithm_libraries`, one load attempt per entry, each individually
  caught and recorded per point 4) and the single **fixed** component
  (`algorithm_so_file` in comparative mode, `mission_control_so_file` in competition mode, loaded
  once via `Registrar::loadMappingAlgorithm`/`loadMissionControl` before the per-component loop
  even starts). Wherever a test below calls `runComparative`/`runCompetition` **more than once** —
  comparing two invocations directly, or repeating one "several times" — **every** `.so` path
  involved in each separate invocation, the fixed component included, must be its own freshly
  `std::filesystem::copy_file`'d copy that no earlier call in the same test process has already
  loaded. A collision on the fixed component is worse than one on a looped component: per point
  4's design, a fixed-component load failure propagates straight out of `runComparative`/
  `runCompetition` uncaught (it's not caught and recorded like a looped component's failure), so
  reusing the same `algorithm_so_file`/`mission_control_so_file` path across two invocations in one
  test makes the *second* invocation throw immediately, before it ever reaches its per-component
  loop — not merely fail one component the way a reused looped path would. Never reuse the same
  scratch folder/paths — looped or fixed — across multiple `runComparative`/`runCompetition` calls
  in one test.

  1. **Concurrent multi-component run produces the same deterministic order as sequential.**
     Build **two separate, complete scratch setups** — each with its own freshly-copied looped
     `.so` files *and* its own freshly-copied fixed `algorithm_so_file` copy, per the constraint
     above (reusing either one across the two calls below breaks this test): each setup gets its
     own scratch `mission_control_folder` containing several (e.g. 6) copies of
     `MissionControl_322889890_315113738.so` under out-of-alphabetical-order filenames (same
     technique `ComparativeMultiComponentGarbageNestingAndOrdering` already uses) and its own
     `std::filesystem::copy_file`'d copy of `Algorithm_322889890_315113738.so` for `algorithm=` —
     one full setup for a `runComparative` call with `options.num_threads = 1` (or unset), the
     other for a call with `options.num_threads = 4` — and assert both runs' `comparative_report.yaml`
     `results_summary`/`errors` structure and ordering are identical (component identity is
     compared by filename, which is identical across the two setups by construction), and that
     every expected `simulation_output_<stem>.yaml` file exists in both. Repeat the
     `num_threads = 4` case several more times, **each against its own fresh, complete scratch
     setup — looped files and the fixed algorithm copy, all newly-copied**, and confirm the
     ordering is identical every time (proves determinism isn't a fluke of one particular
     scheduling).
  2. **Fewer components than requested threads doesn't crash or hang.** 2 components,
     `num_threads = 8`: confirm `computeWorkerCount` reasoning holds in practice (2 worker
     threads actually used, not 8) — assert on wall-clock-independent evidence only (both
     components ran, no crash, no hang/timeout) since actual thread count isn't directly
     observable from outside; the `computeWorkerCount` unit test above already covers the number
     itself.
  3. **Run/write-phase failure still isolates exactly one component under real concurrency** —
     port `ComparativeRunWritePhaseFailureIsolatesOneComponent`'s scenario (one component's
     `results_dir / component_stem` pre-occupied by a regular file) into a multi-component,
     `num_threads >= 2` run with several other healthy components alongside it; confirm the
     blocked one lands in `errors:` and every other component still completes and lands in
     `results_summary:`, exactly as the existing sequential test already proves for the
     sequential path.
  4. **`error.log` under concurrency is complete, non-interleaved, and correctly attributed** (the
     three properties required under concurrency per the "Hard requirements" clarification above —
     **not** line order, which this test must not assert on). Several components (mix of healthy
     and garbage `.so`), `num_threads >= 2`, capture `error.log`'s content via a real temporary
     file this time (not just an in-memory buffer, to also exercise `CerrSinkGuard` wrapping a real
     `std::ofstream`'s buffer). **Scope the `std::ofstream`/`CerrSinkGuard` pair so both are fully
     destroyed before the test opens a separate `std::ifstream` to read `error.log` back** — e.g. in
     an inner block, or a helper function that returns only after both go out of scope. Stage 1
     deliberately performs no per-line flush, so reading the file while either is still alive risks
     observing a truncated or empty file; final on-disk content is only guaranteed once
     `error_log`'s destructor has actually run (see Stage 1's declaration-order note). Then assert:
     (a) every non-empty line matches an expected shape (e.g. starts with `[component=`, or is one
     of the known top-level messages) — i.e. no line is a corrupted concatenation of two
     components' messages (non-interleaving); (b) each garbage/failing component's expected message
     text appears exactly once, attributed to the correct `[component=...]` prefix (completeness +
     attribution); (c) explicitly do **not** assert anything about which component's lines appear
     first. Run the scenario two or three more times, **each against its own fresh, complete
     scratch setup — looped `.so` copies and the fixed-component copy, all newly-copied** — per
     the constraint above (never the same paths, looped or fixed, reused across these repeats).

     **Do not compare full line text for exact equality across repeats.** Because each repeat's
     `.so` copies live in a distinct, freshly-generated scratch directory, a failure message that
     embeds the failing path — `Registrar`'s own `PLUGIN_NOT_REGISTERED` message
     (`library_path.string() + " did not register a..."`) and `dlerror()`'s text for a genuinely
     malformed `.so` (`PLUGIN_LOAD_FAILED`) both do this — will legitimately contain different text
     between repeats even though the underlying behavior is equivalent; asserting exact
     line-for-line equality here would fail on a correct implementation, not catch a real bug.
     Instead, for each repeat independently, assert (a) the **set of component *names*** —
     `[component=X]`'s `X`, always a bare filename via `library_path.filename().string()`, never a
     full path, so stable across repeats by construction — that appear as load/run failures matches
     the expected set of healthy/garbage components for that scenario, and (b) each such
     component's line contains the expected *stable* portion of its failure reason (e.g. the
     literal substring `"failed to load "`, or the `PLUGIN_LOAD_FAILED`/`PLUGIN_NOT_REGISTERED`
     code), deliberately excluding whatever scratch-path text `dlerror()`/`Registrar` embedded
     after it. This is what "the same every time" means for this test in the presence of
     per-repeat-unique scratch paths: component identity and failure category must be
     repeat-invariant; the literal byte-for-byte message text is not, and must not be asserted on.
  5. **`std::size_t runComparative(...)`/`runCompetition(...)`'s return value** (count of
     components that ran successfully) is correct in a concurrent multi-component scenario with a
     mix of successes and failures — should already fall out of test 1/3 above, but assert it
     explicitly.

**Must not change:** `SimulationManager.*`, `SimulationOutputWriter.*`, `SimulationRunFactoryImpl.*`,
`ConfigLoader.*`, `Registrar.*`, `ISimulationRun.h`, `ISimulationRunFactory.h`, `CliOptions.*`,
anything under `common/`/`Algorithm/`/`MissionControl/`. `SimulatorRunner.h`'s public signatures
(`runComparative`/`runCompetition`) are unchanged.

---

## Stage 3 — Integration verification, stress testing, and documentation

**Precondition:** Stages 1 and 2 complete and their own verification steps passing.

**Build and test only inside the project's Linux devcontainer/vcpkg `x64-linux` toolchain** (per
`RULES.md` — `dlopen`/`.so`/`ENABLE_EXPORTS` are POSIX/ELF-only; this cannot be built or run on
native Windows).

1. **Full rebuild and full existing suite, unchanged pass rate:**
   ```bash
   cmake --build build/default --target simulator_322889890_315113738 simulator_registration_test
   build/default/bin/simulator_registration_test
   build/default/bin/simulator_registration_test --gtest_shuffle
   ```
   All suites (`RegistrationEndToEnd`, `Stage1Verify`, `Stage2Verify`, `Stage3Verify`, this plan's
   new `CerrContextVerify`/`cerr_context_verify` suite, `SimulatorRunnerVerify`, and
   `MultithreadingVerify`) must pass, including under `--gtest_shuffle` (order-independence,
   same reasoning `SIMULATOR_CORE_PLAN.md`'s Stage 4 cleanup already established for the
   `Registrar` singleton's process-lifetime state — this plan doesn't change that hazard, just
   confirm nothing new was introduced).

2. **Real end-to-end runs, both modes, with and without `num_threads`:** using the same
   `inputs/sim_compose_small.yaml` fixture `SIMULATOR_CORE_PLAN.md`'s Stage 4 established (fast,
   real pipeline execution, non-`-1` scores):
   ```bash
   rm -rf /tmp/mc_folder_mt && mkdir -p /tmp/mc_folder_mt
   for i in 1 2 3 4 5 6; do cp build/default/bin/MissionControl_322889890_315113738.so /tmp/mc_folder_mt/mc_$i.so; done
   build/default/bin/simulator_322889890_315113738 -comparative \
     simulation=inputs/sim_compose_small.yaml \
     mission_control_folder=/tmp/mc_folder_mt \
     algorithm=build/default/bin/Algorithm_322889890_315113738.so \
     num_threads=4
   ```
   **Always `rm -rf` a scratch component folder immediately before `mkdir -p`-ing and populating
   it — for every folder in this step and its `-competition` counterpart below, never just
   `mkdir -p` on its own.** These commands are meant to be re-run as many times as needed while
   debugging; `mkdir -p` alone is a no-op on an already-existing directory and would silently leave
   whatever `.so` files an earlier, different verification attempt left there (e.g. a previous pass
   that copied a different count, or an older build's stale copy of the same file) — changing the
   component count and report content without it being obvious why, unlike the automated tests'
   `ScratchDir` helper (`stage3_verify_test.cpp`), which already does exactly this
   `remove_all`-then-`create_directories` sequence for the same reason.

   Confirm exit 0, `comparative_results_<...>/` populated with all 6 `simulation_output_mc_*.yaml`
   files plus `comparative_report.yaml` plus `error.log`, and — the actual point of this run —
   `rm -rf /tmp/mc_folder_mt2 && mkdir -p /tmp/mc_folder_mt2`, copy the same 6 `.so` files into it,
   run the identical command again with `num_threads` **omitted** against that second folder, and
   diff the two runs' `comparative_report.yaml` `results_summary`/`errors` structure (content
   should match exactly — `generated_at_utc` will legitimately differ, ignore that one field) to
   directly confirm sequential vs. concurrent execution of the same inputs produce the same report.
   Do **not** diff the two runs' `error.log` files line-for-line — per the "Hard requirements"
   clarification above, line order across components is expected to differ between the two runs;
   if you want to sanity-check `error.log` here too, compare it only as an unordered set of
   expected message lines (there should be none for this healthy fixture beyond whatever this
   codebase already logs unconditionally, if anything). Repeat symmetrically for `-competition`
   with several `Algorithm_*.so` copies under a freshly `rm -rf`'d-then-recreated
   `/tmp/algo_folder_mt`.

3. **Stress/race check — strongly recommended, not just a nice-to-have given this plan's whole
   point is eliminating data races:** if the devcontainer's toolchain supports it, configure a
   second, throwaway build directory with `-fsanitize=thread` added to
   `target_compile_options`/`target_link_options` for **three** targets, not just one —
   `simulator_registration_test`, **and also** `Algorithm_322889890_315113738` and
   `MissionControl_322889890_315113738`. All three are required: those two plugin targets build
   the `.so` files `simulator_registration_test` `dlopen`s at runtime (the fixed
   `ALGORITHM_PLUGIN_PATH`/`MISSION_CONTROL_PLUGIN_PATH` fixtures, and every copy this plan's own
   new tests make of them), and they are separate compilation units built by their own CMake
   targets (`add_library(... SHARED ...)` in `Algorithm/CMakeLists.txt`/
   `MissionControl/CMakeLists.txt`, only linked to the test target via `add_dependencies` for build
   ordering — no compile-flag inheritance at all). ThreadSanitizer only instruments memory accesses
   in code actually compiled with `-fsanitize=thread`; a race entirely inside
   `MappingAlgorithmImpl.cpp`/`MissionControlImpl.cpp`/`DroneControlImpl.cpp` would be completely
   invisible to a TSan run that only instrumented the test binary, even though that is exactly the
   code the "Shared-state audit" section above relies on staying race-free (the fixed component's
   factory is invoked concurrently by every worker thread once Stage 2 lands). Build all three with
   the flag (do not leave `-fsanitize=thread` enabled in the submitted `CMakeLists.txt` for any of
   them), then run: `build-tsan/bin/simulator_registration_test`. ThreadSanitizer should report
   zero races. If it flags anything inside this plan's own new code, or inside the two
   now-instrumented plugin `.so`s, treat it as a correctness bug to fix, not noise to suppress.

   **Known, accepted limitation of this check — no further action needed:** vcpkg-provided
   third-party libraries (`yaml-cpp`, `TinyNPY`, `mp-units`, `gtest`) are linked as prebuilt
   binaries and will not be rebuilt with `-fsanitize=thread` by this throwaway configuration (doing
   so would require a custom vcpkg triplet, out of scope for a one-off local check). Accesses
   inside those libraries stay invisible to this run the same way an uninstrumented plugin `.so`
   would have been — this TSan run does **not** dynamically confirm the "Shared-state audit"
   section's reasoning about `TinyNPY::LoadNPY`/yaml-cpp's `Node` construction; that reasoning, and
   the spot-check it already asks for, remains the only coverage for that specific risk. If TSan
   happens to flag something that traces into one of these libraries anyway, add the targeted
   mutex that section already describes rather than disabling the sanitizer run.

**Must not change:** everything listed as "must not change" in Stages 1 and 2, plus: do not leave
any `-fsanitize=thread` flag enabled in the checked-in `CMakeLists.txt` after this stage — that
build variant is a one-off local verification step, not part of the submission.

---

## Summary of files touched by this plan

- `Simulator/include/Simulator/CerrContextGuard.h` — rewritten internals (Stage 1).
- `Simulator/src/CerrContextGuard.cpp` — rewritten internals (Stage 1).
- `Simulator/src/drone_mapper_simulation_main.cpp` — swap `CerrRedirectGuard` for
  `simulator::CerrSinkGuard` (Stage 1).
- `Simulator/tests/stage3_verify/stage3_verify_test.cpp` — fix `CerrCapture` helper only; no
  test-case logic changes (Stage 1).
- `Simulator/tests/cerr_context_verify/cerr_context_verify_test.cpp` — new (Stage 1).
- `Simulator/src/SimulatorRunner.cpp` — rewritten `runComparative`/`runCompetition` bodies, new
  `computeWorkerCount`/`runIndexed`/`ComponentOutcome` helpers (Stage 2).
- `Simulator/include/Simulator/SimulatorRunner.h` — add the `computeWorkerCount` declaration only;
  `runComparative`/`runCompetition`'s existing public signatures are unchanged (Stage 2).
- `Simulator/tests/multithreading_verify/multithreading_verify_test.cpp` — new (Stage 2).
- `Simulator/CMakeLists.txt` — add the two new test `.cpp` files to `simulator_registration_test`'s
  source list (Stages 1 and 2); no other CMake changes.
- `README.md` (submission root) — short design note (Stage 3).

**Never touched by this plan:** `common/` (anything), `Algorithm/` (anything), `MissionControl/`
(anything), `Registrar.h`/`.cpp`, `SimulationManager.h`/`.cpp` (besides the `CerrContextGuard`
`#include` it already has from `SIMULATOR_CORE_PLAN.md`'s Stage 3 — no further change), `ConfigLoader.*`,
`SimulationOutputWriter.*`, `SimulationRunFactoryImpl.*`, `SimulationRunImpl.*`, `Map3DImpl.*`,
`Mock*.*`, `MapsComparison.*`, `CliOptions.*`, `SimulatorRunner.h`'s public signatures,
`MappingAlgorithmRegistration.cpp`/`MissionControlRegistration.cpp`.
