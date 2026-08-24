# Threading Refactor Plan — per-(component, run) scheduling

**Decision (final):** the parallel unit of work is one `(component, simulation-run)` pair.
`component` = one `MissionControl_*.so` (comparative mode) or `Algorithm_*.so` (competition mode).
`simulation-run` = one `simulation × mission × drone × lidar` combination, i.e. one entry of the
nested loop currently in `SimulationManager::runInternal`. Total task count for one invocation =
`valid_component_count × combinations_per_composition`.

This file is implementation-ready. No code has been changed yet.

## Files / functions touched

- `Simulator/include/Simulator/SimulationManager.h`, `Simulator/src/SimulationManager.cpp` —
  split `runInternal`'s nested loop into two reusable pieces: `enumerateRuns` (combination
  enumeration + leaf-dir naming, no factory involved) and `runOne` (build-and-execute one
  combination, the existing try/catch → `buildErrorResult` body). `SimulationManager::run(...)`
  keeps its current public signature and behavior, reimplemented as `enumerateRuns` + a sequential
  loop over `runOne` — every existing caller (tests included) is unaffected.
- `Simulator/src/SimulatorRunner.cpp` — `runComparative`, `runCompetition`, `ComponentOutcome`,
  `runOneComponent` (removed), `computeWorkerCount`'s call sites, `runIndexed`'s call sites. See
  "Scheduling flow" below.
- `Simulator/include/Simulator/SimulatorRunner.h` — no signature changes; `computeWorkerCount`'s
  existing `work_items` parameter now receives a different value (total tasks, not component
  count) — document that in its comment.
- **Not touched:** `Registrar.*`, `CerrContextGuard.*`, `SimulationOutputWriter.*`,
  `SimulationRunFactoryImpl.*`, `SimulationRunImpl.*`, `ConfigLoader.*`, `CliOptions.*`. `runIndexed`
  itself (the atomic-cursor dispatch loop) is untouched — it already runs `process(i)` for
  `i in [0, item_count)` off a shared cursor, agnostic to what `i` means.

## New task model and scheduling flow

`SimulationManager.h` gains:

```cpp
struct RunSpec {
    std::size_t sim_index, mission_index, drone_index, lidar_index;
    std::filesystem::path relative_leaf_dir; // e.g. "simulations/sim_1/mission_a/drone_small__lidar_a"
};

// Enumerates every simulation×mission×drone×lidar combination once, in the same deterministic
// order runInternal's nested loop uses today. Pure w.r.t. component identity: relative_leaf_dir
// does not depend on which component will run it, so this is computed exactly once per
// SimulatorRunner call and shared read-only across every component.
[[nodiscard]] std::vector<RunSpec> enumerateRuns(
    const types::SimulationCompositionData& composition,
    const CompositionFilePaths& file_paths);

// Builds and executes exactly one combination: identical logic to today's runInternal loop body
// (sim_ref/mission_ref load_error -> buildErrorResult, else run_factory.create(...)->run(),
// exceptions caught and converted to buildErrorResult). component_root is
// results_dir / component_stem; this function joins it with spec.relative_leaf_dir and calls
// create_directories itself, same as today.
[[nodiscard]] types::SimulationResult runOne(
    ISimulationRunFactory& run_factory,
    const types::SimulationCompositionData& composition,
    const CompositionFilePaths& file_paths,
    const RunSpec& spec,
    const std::filesystem::path& component_root);
```

`SimulatorRunner.cpp`'s `ComponentOutcome` gains a per-run results slot:

```cpp
struct ComponentOutcome {
    std::string component_name;
    std::vector<types::SimulationResult> runs; // sized to run_specs.size(); one slot per RunSpec
    std::optional<ComponentRunTotals> totals;  // set only after successful assembly
};
```

`runComparative` (symmetric for `runCompetition`, swap which factory is fixed vs. looped):

1. `parseCompositionData` — unchanged.
2. Load the fixed factory (`algorithm_so_file` / `mission_control_so_file`) — unchanged, still
   single load before anything else.
3. `sortedByFilename(component libraries)` — unchanged.
4. **New:** `const std::vector<RunSpec> run_specs = enumerateRuns(parsed.composition, parsed.file_paths);`
   — once, single-threaded, before any dispatch.
5. `std::vector<ComponentOutcome> outcomes(component_libraries.size());` each `runs` resized to
   `run_specs.size()`.
6. **New preload pass — single-threaded, sequential, one iteration per component:**
   - `outcomes[i].component_name = library_path.filename().string();`
   - `try { factory = Registrar::instance().loadMissionControl(library_path); }` — on failure, log
     and leave `outcomes[i].totals` unset (recorded as a failure at assembly time); this component
     contributes zero tasks.
   - On success, build `auto run_factory = std::make_shared<SimulationRunFactoryImpl>(mapping_algorithm_factory, factory, options.verbose);`
     and push `(component_index=i, run_factory)` into a `std::vector` of valid components.
7. `const std::size_t num_runs = run_specs.size();`
   `const std::size_t total_tasks = valid_components.size() * num_runs;`
   `const std::size_t worker_count = computeWorkerCount(options.num_threads, total_tasks);`
8. `runIndexed(total_tasks, worker_count, [&](std::size_t task_index) { ... })`:
   - `const std::size_t slot = task_index / num_runs; const std::size_t run_index = task_index % num_runs;`
   - `const auto& [component_index, run_factory] = valid_components[slot];`
   - `const CerrContextGuard component_guard("component=" + outcomes[component_index].component_name);`
   - `outcomes[component_index].runs[run_index] = runOne(*run_factory, parsed.composition, parsed.file_paths, run_specs[run_index], results_dir / component_stem(component_index));`
9. **New single-threaded assembly pass, after `runIndexed` returns**, iterating `outcomes` in
   original (sorted) component order — same loop shape as today's `aggregateOutcomes`:
   - Component never loaded → `failures.push_back(outcomes[i].component_name)`.
   - Component loaded → build `types::SimulationManagerReport{parsed.composition.composition_file, currentUtcTimestamp(), kMetric, {kScoreRangeMin, kScoreRangeMax}, kErrorScore, std::move(outcomes[i].runs)}`,
     call `writeSimulationOutput(report, parsed.file_paths, results_dir / ("simulation_output_" + component_stem + ".yaml"))`,
     `totals.push_back(computeComponentTotals(outcomes[i].component_name, report))`.
10. `writeComparativeReport(...)` — unchanged call, same inputs shape (`totals`, `failures`).

## `.so` loaded once; fresh instances created per run

- Every `Registrar::loadMissionControl` / `loadMappingAlgorithm` call now happens **only** in step 6
  (the single-threaded preload pass) — never from inside a task. This is strictly safer than today,
  where each worker loaded its own component lazily; now no load ever races with another load or
  with a run.
- One `SimulationRunFactoryImpl` is built per component in step 6 and shared (via `shared_ptr`)
  across every task for that component. It is immutable after construction (two `Factory` copies +
  a `bool`, no mutable members — confirmed by reading `SimulationRunFactoryImpl.h`), so concurrent
  `create()` calls from multiple threads against the same instance are safe.
- Every `create()` call (inside `runOne`, unchanged from today) still builds a **fresh**
  `SimulationRunImpl` owning fresh `unique_ptr`s to a new `MissionControl`/`MappingAlgorithm`/`GPS`/
  `Lidar`/`Movement`/hidden-map/output-map for that one run — confirmed by reading
  `SimulationRunImpl.h`'s constructor. Nothing here changes; it's the existing per-run behavior,
  now reachable from many concurrently-running tasks against the same component instead of from one
  thread that owned that whole component.

## Results / output ordering preserved

- `outcomes` stays indexed by each component's position in the already-`sortedByFilename` list —
  unchanged from today.
- Within one component, `outcomes[component_index].runs[run_index]` is written by exactly one task
  (each `(slot, run_index)` pair is produced by exactly one `task_index`), so no lock is needed and
  the `runs` vector always ends up in `run_specs` order regardless of task completion order —
  matches `enumerateRuns`' deterministic enumeration order, which matches today's nested-loop order.
- The assembly pass (step 9) walks `outcomes` in fixed component order and reassembles
  `totals`/`failures` exactly as today's `aggregateOutcomes` does — `results_summary`/`errors`
  ordering in the YAML reports is unaffected by which task finished first.
- `error.log` line order across different components/runs is **not** required to match any fixed
  order once `num_threads >= 2` (unchanged expectation from before); completeness, non-interleaving,
  and correct `[component=...] [sim=... mission=... drone=... lidar=...]` attribution are what must
  hold, and are what `CerrContextGuard`'s thread-local label stack + `CerrSinkGuard`'s mutex-guarded
  writer already guarantee (see audit below).

## Thread-safety audit — concurrent run path, focused on same-`.so` concurrency

This is the one genuinely new condition versus before: multiple runs of the **same** loaded `.so`
can now execute simultaneously on different threads (previously each component was owned by exactly
one thread for its whole composition sweep).

- **`Registrar`** (`Registrar.h`): `load_mutex_`-guarded, returns a `Factory` by value. No longer
  called from any task at all under this design (see above) — not a concern here regardless.
- **`SimulationRunFactoryImpl`**: stateless after construction (checked above) — safe to share one
  instance across concurrent `create()` calls for the same component.
- **The plugin's own factory function** (the `std::function` a submitted `.so`'s `REGISTER_*` macro
  produces): this codebase's own `Algorithm`/`MissionControl` implementations were previously audited
  as captureless-lambda, no shared mutable state. **New risk surface under this design:** a
  *different team's* submitted `.so` could contain static/global mutable state inside its
  `IMissionControl`/`IMappingAlgorithm` implementation, which this refactor now calls concurrently
  for the same component far more often (every run of that component, not just one call). This is
  not something the Simulator can fix — flag it as a known limitation, not a blocking item.
- **`SimulationRunImpl`**: every run constructs entirely fresh dependency instances (`unique_ptr`
  members, no statics) — confirmed safe for concurrent runs of the same component by construction.
- **`TinyNPY`'s `LoadNPY`** (hidden-map loading) and **`yaml-cpp`'s emitter** (`writeSimulationOutput`):
  previously only needed to tolerate concurrent calls for *different* files/components. Under this
  design, two runs of the **same component with the same `map_filename`** can call `LoadNPY`
  concurrently on the identical file — a strictly stronger requirement. Re-verify (read the vendored
  source, don't assume) that `LoadNPY` has no shared static buffer/error-message pointer before
  relying on this.
- **`std::filesystem::create_directories`**: two runs of the same component can now share a parent
  directory prefix concurrently (e.g. same `sim`/`mission`, different `drone`/`lidar`, both created
  at nearly the same time) — this did not happen before (one thread, one component, strictly
  sequential creation). POSIX `mkdir` is atomic per call and `create_directories` implementations
  generally tolerate a concurrently-created intermediate directory, but this must be verified against
  the actual standard library in use (not assumed) and exercised by a real concurrent test (below),
  not just trusted.
- **`CerrContextGuard`/`CerrSinkGuard`**: already thread-local-stack-based with a single
  mutex-guarded flush on `'\n'` — already handles multiple threads holding *different* labels
  concurrently. This design newly exercises multiple threads holding the **same**
  `"component=X"` label concurrently (several runs of one component in flight at once) — the
  thread-local stack design has no dependency on labels being unique across threads, so this should
  already be correct; call it out as a scenario to explicitly cover in tests, not as a design gap.
- **`outcomes[component_index].runs[run_index]`**: disjoint-index writes, vector sized once before
  dispatch, never resized during `runIndexed` — no lock needed, same reasoning already used for the
  existing `outcomes[index]` pattern.

## Tests needed

1. **`computeWorkerCount` regression + the bug this refactor fixes:** add
   `computeWorkerCount(8, /*total_tasks=*/1 * 20) == 8` — today's equivalent
   (`computeWorkerCount(8, /*component_count=*/1)`) returns `0`; this is the concrete case that
   motivated the refactor and must now parallelize.
2. **`enumerateRuns` unit test:** given a composition fixture, confirm the returned `RunSpec` list's
   order and `relative_leaf_dir` values exactly match today's sequential `runInternal` output
   (regression against existing `stage2_verify`/`stage3_verify` composition fixtures).
3. **`runOne` unit test:** given one `RunSpec` + a stub `ISimulationRunFactory`, confirm identical
   output to today's per-iteration loop body, including the `sim_ref`/`mission_ref` `load_error` →
   `-1`-score path.
4. **Multi-thread utilization proof:** 1 component, composition with ≥ 8 combinations,
   `num_threads=8`. Use a test double `ISimulationRunFactory`/`ISimulationRun` that records the
   calling thread's ID. Assert more than one distinct thread ID was used — directly proves the
   original single-component gap is closed.
5. **Same-component concurrency stress test:** 1 component, composition sized so several runs share
   the same `map_filename` and/or the same `sim`/`mission` directory prefix, `num_threads` high,
   repeat several times (and under `-fsanitize=thread` if the toolchain supports it). Assert: no
   crash/race, every run produces its expected output file in its expected directory, no directory
   creation failure.
6. **Deterministic ordering under concurrency:** run the same multi-component composition several
   times with `num_threads >= 2`; assert `comparative_report.yaml`/`competitive_report.yaml`
   `results_summary`/`errors` ordering is identical across repeated runs even though per-run
   completion order varies.
7. **Byte-identical sequential path:** `num_threads` absent/`1` must still produce output identical
   to the pre-refactor implementation — re-run existing `stage2_verify`/`stage3_verify` suites
   unchanged and confirm they pass without modification.
8. **Failure isolation:** one component fails to load (recorded in `failures`, contributes zero
   tasks) while a run in a *different* component throws (recorded as a `-1` result via
   `buildErrorResult`) — confirm both are handled independently and every other task still
   completes.
9. **`error.log` attribution under same-component concurrency:** extend the existing
   `cerr_context_verify` test with a case where multiple threads hold the *same* `"component=X"`
   label concurrently (differing only by `sim=.../mission=.../...`); assert completeness,
   non-interleaving, and correct per-line attribution, same criteria as the existing test's
   cross-component case.

## Implementation order

1. Extract `enumerateRuns` + `runOne` from `SimulationManager::runInternal`; reimplement
   `SimulationManager::run(...)` on top of them. Re-run existing `stage2_verify`/`stage3_verify`/
   `simulation_manager_test` unchanged — must still pass byte-for-byte.
2. Extend `ComponentOutcome` with `runs`; remove `runOneComponent`.
3. Add the single-threaded preload pass to `runComparative`/`runCompetition`.
4. Wire `computeWorkerCount`'s call site to `valid_components.size() * run_specs.size()`.
5. Replace the per-component `runIndexed` call with the flattened `(component, run)` dispatch and
   index decode.
6. Add the post-`runIndexed` single-threaded assembly pass (build `SimulationManagerReport`, call
   `writeSimulationOutput`/`computeComponentTotals`, populate `totals`/`failures`).
7. Add/extend the tests listed above; run full suite, including a TSAN build if available.
8. Grep `README.md`/`CHECK_LIST.md` for descriptions of the old per-component-only threading model
   and update them to match.

## Acceptance checklist

- [ ] `computeWorkerCount` is called with `valid_component_count × run_count`, not component count
      alone.
- [ ] 1 component with many composition combinations and `num_threads >= 2` spawns more than one
      worker thread (test 4).
- [ ] `num_threads` absent/`1` produces output byte-identical to the pre-refactor sequential run
      (test 7).
- [ ] Worker count is never exactly `1`; total thread count is never exactly `2` — re-verified
      against the new `work_items` value, same formula as before.
- [ ] Every `.so` is `dlopen`'d exactly once per process; no `Registrar::load*` call happens outside
      the single-threaded preload pass.
- [ ] Every `(component, run)` task produces a fresh `MissionControl`/`Algorithm`/GPS/Lidar/Movement/
      map instance set (unchanged `SimulationRunFactoryImpl::create` behavior).
- [ ] `comparative_report.yaml`/`competitive_report.yaml` ordering is deterministic and
      component-order-based, independent of task completion order (test 6).
- [ ] Concurrent runs of the same component sharing a map file / directory prefix complete correctly
      with no data race (test 5; TSAN clean if available).
- [ ] `error.log` stays complete, non-interleaved, and correctly attributed under same-component
      concurrency (test 9).
- [ ] All existing `stage2_verify`/`stage3_verify`/`multithreading_verify`/`cerr_context_verify`
      tests pass, unchanged or updated exactly per this plan.
