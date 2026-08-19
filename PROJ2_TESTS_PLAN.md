# Project 2 → Project 3 Test Migration & Coverage Plan

Status: **reference/analysis document.** Phases 1–4 of the migration are complete
(Project 2 test validation/repair, migration manifest, Project 3 test
infrastructure, and component-test migration). Phase 5 (Project 3-specific
SIM16/SIM19 regression coverage) is next. See `PROJ2_TESTS_EXECUTION_PLAN.md`
for live phase status and `_migration_staging/MIGRATION_MANIFEST.md` for the
finalized per-file migration inventory.

---

## 1. Purpose & Scope

We have 260 GTest/GMock tests in Project 2 (`FILES PROJECT 2/tests/**`, post-Phase-1;
was ~253 pre-Phase-1) and, prior to Phase 3, had effectively zero behavioral tests in
Project 3 (`ex_3_skeleton-main` only had skeleton "verify" tests — see §3; Phase 3 has
since stood up real component-test targets for `MissionControl` and `Algorithm`, see
§6). This plan defines:

1. Which Project 2 tests are still valid and how to carry them into Project 3 with
   minimal semantic drift, additively (never replacing existing Project 3 tests).
2. A safe **validate-in-Project-2-first** workflow for repairing/adding tests before
   they ever touch Project 3.
3. The mutation-coverage gaps the staff feedback exposed, triaged by actual risk
   instead of raw bug count.
4. Why `LID03` and `DRO08` produced **integration timeouts**, and how to catch them
   with fast, deterministic component tests added *alongside* the existing
   integration tests — without rewriting, narrowing, or weakening the latter.
5. A regression test plan for the `MissionRunStatus::Error → mission_score == -1`
   fix that has already been applied manually in Project 3 — written directly
   there, not through the Project 2 workflow (see §4's `SIM16` carve-out).

---

## 2. Sources Reviewed

- `FILES PROJECT 2/tests/**` — all component, integration, internal, and audit tests
  (15 files post-Phase-1, 260 `TEST`/`TEST_F` cases — full per-file inventory in §5;
  was 14 files/~253 cases pre-Phase-1).
- `FILES PROJECT 2/src/**`, `FILES PROJECT 2/include/**` — the submitted implementation
  the tests above exercise.
- `FILES PROJECT 2/EXSTRA FILES/feedback.txt` — coverage results (`component coverage`
  / `integration coverage` / `integration timeouts` bug-ID lists).
- `FILES PROJECT 2/EXSTRA FILES/Exercise 2 - Grading Explanation.docx` — grading
  methodology, exceptions list (`LID04`, `MIS18`/`ALG28`, `COM24`, `DRO07`/`SIM20`).
- `FILES PROJECT 2/EXSTRA FILES/mutated_src/**` — the actual `#ifdef <BUG>` mutation
  points for all 28 bugs; read directly (not just the bug catalogue) to see exactly
  which line each mutation touches and what the *default* (unmutated) branch does.
- The staff bug catalogue (linked from the grading doc) — one-line description of all
  28 bugs (`LID01…LID05`, `DRO06…DRO12`, `MIS14…MIS18`, `SIM16/19/20`, `MAN21/22/23/27`,
  `COM24/25/26`, `ALG28/29`).
- `ex_3_skeleton-main/**` — current Project 3 source, headers, and CMake targets, to
  determine the real (not assumed) structural differences from Project 2.

## 3. Architecture Delta: Project 2 → Project 3

This is the most important input to the plan — nothing here is a 1:1 rename job.

| Aspect | Project 2 | Project 3 |
|---|---|---|
| Build shape | One binary, everything statically linked (`drone_mapper` lib + `drone_mapper_simulation_test`) | Multiple CMake targets: `Simulator` (exe), `MissionControl` (**SHARED lib**, loaded via registration/plugin macro), `Algorithm` (**SHARED lib**, same), `common` (header-only interface lib) |
| Namespace | Everything in `drone_mapper` | `common` / `common::types` (shared interfaces & data types), `simulator` (Simulator-local: `SimulationRunImpl`, `SimulationManager`, `MapsComparison`, `Map3DImpl`, `MockGPS/Lidar/Movement`, `ConfigLoader`), `MissionControl_322889890_315113738` (`MissionControlImpl`, `DroneControlImpl`, `ScanResultToVoxels`), Algorithm plugin (`MappingAlgorithmImpl`) |
| `IMissionControl` construction | `MissionControlImpl(mission, drone, hidden_map, output_map, drone_control, output_map_file)` — `IDroneControl` **injected** from outside, mockable directly | `MissionControlImpl(MissionControlDependencies)` — an aggregate struct; `DroneControlImpl` is now constructed **internally** by `MissionControlImpl`, no external injection seam |
| `IDroneControl` | Public interface in `include/drone_mapper/IDroneControl.h`, used by `SimulationRunImpl` too | Interface now lives at `MissionControl/common_mission_control/include/MissionControl/IDroneControl.h` (module-local, namespace `mission_control`); **not** referenced by `Simulator` at all anymore |
| `DroneControlImpl` dependencies | `ILidar&, IGPS&, IDroneMovement&, IMutableMap3D&, IMappingAlgorithm&` — unchanged in P3 | Same interfaces, same by-reference shape — **this seam is intact**, `drone_control_test.cpp` migrates almost verbatim |
| `SimulationRunImpl` dependencies | 8 injected `unique_ptr` interfaces incl. `IDroneControl`, `IMissionControl` | `hidden_map, output_map, gps, movement, lidar, mapping_algorithm, mission_control` — **no `IDroneControl` parameter** (dropped; MissionControl owns its own drone control now) |
| Config types | `MissionConfigData` / `SimulationConfigData` carried a `load_error` field (Project 2 interface deviation, already covered in the separate code-review pass) | `load_error` moved out into a Simulator-local `ReferencedConfigFile` type (`Simulator/include/Simulator/ConfigLoader.h`) — **already fixed**, not a test-migration concern |
| Test infra today | `tests/components/*.cpp` + `tests/integration/*.cpp` globbed into one `drone_mapper_simulation_test` GTest+GMock binary | Prior to Phase 3, only the Simulator verify/registration tests existed. Phase 3 has since added dedicated component-test infrastructure for MissionControl, Algorithm, and Simulator, including the required module-local mocks and test-only registration stubs. See §6 and `PROJ2_TESTS_EXECUTION_PLAN.md` Phase 3. |

**Consequence for this plan:** migration is not pure search-and-replace. Three concrete
structural adaptations are required and are called out per-component in §7:

1. `mission_control_test.cpp` (which mocked `IDroneControl` to unit-test
   `MissionControlImpl` in isolation) has **no equivalent seam** in Project 3 anymore.
   It must be re-scoped to test `MissionControlImpl` + real `DroneControlImpl` together,
   injecting mocks one level down (`ILidar`, `IGPS`, `IDroneMovement`,
   `IMappingAlgorithm`, `IMutableMap3D`) instead of mocking drone control out entirely.
2. `simulation_run_test.cpp` fixtures that build a `SimulationRunImpl` with a mocked
   `IDroneControl` argument must drop that argument — the constructor signature is
   shorter in Project 3.
3. Dedicated CMake test targets were required for `MissionControl` and `Algorithm`; Phase 3 has now created and smoke-tested them (§6). Phase 4 uses those established targets for the bulk migration.

---

## 4. Do we agree with "validate in Project 2 first"? — Yes, with one refinement

**Yes.** Reproducing each staff mutation inside Project 2 (which already builds
against exactly the code the test was written for) and confirming red→green there is
strictly safer than writing or fixing a test directly against Project 3, because:

- Project 2 is a known-graded baseline: we already know which mutations it catches and
  misses (`feedback.txt`), so a locally-reproduced red/green result there is directly
  comparable to ground truth.
- Before Phase 3, Project 3 was mid-refactor and did not yet have working component-test targets for `MissionControl`/`Algorithm`. That was why test intent was validated in Project 2 first; Phase 3 has since established and smoke-tested the required Project 3 test infrastructure.
- The `mutated_src` bug flags are already expressed against Project 2's source layout
  (`#ifdef LID03` etc. in `drone_mapper` files); reproducing them in Project 3 first
  would require re-deriving each mutation's new location by hand, with no staff
  reference to check against.

**Refinement:** treat "validate in Project 2" as validating the **test's correctness
and mutation-killing power**, not as validating the **final Project 3 assertions**
line-for-line. Namespaces, constructor shapes, and fixture plumbing *will* differ
(§3) — those differences get resolved during the port step (§7 step 4). We are not
going to hand-port all 28 bug flags into Project 3 source as a matter of course;
that's disproportionate to the goal (safe test migration), and the point of this
plan is to prioritize meaningful behavioral coverage over mutation-count
maximization (per your instruction in §7).

**`SIM16` (and its paired `SIM19` coverage gap) are explicitly excluded from this
Project 2 workflow — do not validate them here.** Every other bug's mutation flag
follows the normal convention (default/off = correct, `-D<BUG_ID>` = injected
fault). `SIM16` inverts that: Project 2's real, unmutated source already has the
bug unconditionally (`result.mission_score = scores[0];`, no status check at all),
and the `mutated_src` flag for it happens to be wired the other way round — turning
`-DSIM16` *on* is what makes the score-on-error behavior correct, not what breaks
it. Getting a red→green cycle for a "score `== -1` on error" test inside Project 2
would therefore require either building with that inverted flag (misleading, and
not representative of how the other 27 bugs work) or literally patching Project 2's
checked-in `SimulationRunImpl.cpp` to add the fix — and Project 2 is meant to stay
the original graded implementation, used only to validate coverage against the
staff mutations as they actually shipped, not to have its own implementation
patched. `SIM16` is already fixed manually in Project 3, so its regression test
(and the adjacent `SIM19` coverage gap, same fixture/scenario) is written directly
against Project 3 during the regression phase — see §10.

### 4.1 Workflow (per test)

```
┌─ Project 2 sandbox (isolated checkout/build dir, never the graded submission) ──┐
│                                                                                  │
│  1. Build baseline:            cmake --build . --target drone_mapper_simulation_test
│     Run the specific suite:    ./drone_mapper_simulation_test --gtest_filter=<Suite>.*
│     → must be 100% green on unmutated code.                                     │
│                                                                                  │
│  2. Rebuild with ONE bug flag: cmake -DCMAKE_CXX_FLAGS="-D<BUG_ID>" --build .    │
│     (or target_compile_definitions on a throwaway target — see §4.2)            │
│     Run the SAME suite again.                                                   │
│     → the new/repaired test must FAIL. If nothing fails, the test doesn't       │
│       actually exercise the mutated code path — fix the test, not the flag.     │
│                                                                                  │
│  3. Flip the flag back off, confirm green again (rules out a flaky/order-       │
│     dependent test that "fails" for the wrong reason).                          │
│                                                                                  │
│  4. Only now: port the verified test into Project 3 (§7), adapting for the      │
│     structural deltas in §3. Tag it as migrated (§7.1).                         │
│                                                                                  │
│  5. Build & run in Project 3. It must pass against current (unmutated) P3       │
│     code. (We are not re-applying `#ifdef` flags inside P3 source — P3 has no   │
│     mutation harness. This step only re-confirms the *ported* test still        │
│     compiles and passes against the real implementation it now targets.)        │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Practical mechanics for step 2

`mutated_src/*.cpp` are **standalone copies**, not the actual `src/*.cpp` with flags
sprinkled in — the real `FILES PROJECT 2/src/*.cpp` files have no `#ifdef` at all. To
reproduce a mutation:

1. Copy the relevant #ifdef <BUG> region from mutated_src into the corresponding FILES PROJECT 2/src file. FILES  PROJECT 2 is already a disposable copy, so no additional sandbox copy is needed. Apply only one mutation at a time and restore the clean source before testing the next mutation.
2. Add the matching `target_compile_definitions(drone_mapper PRIVATE <BUG_ID>)` line
   (or pass `-D<BUG_ID>` on the command line) for that one build.
3. Run only the relevant `--gtest_filter`, not the whole suite, to keep iteration fast.
4. Revert the sandbox copy (or just `git checkout` it back) before touching the next bug.

---

## 5. Test Inventory & Migration Verdict

All counts are `grep -c '^TEST'` on the actual files (includes `TEST` and `TEST_F`),
taken **post-Phase-1** — i.e. reflecting the repairs/additions Phase 1 actually made,
cross-checked against `_migration_staging/MIGRATION_MANIFEST.md` (the authoritative
per-file record). Where a count changed from the original pre-Phase-1 pass, both
numbers are shown.

| Project 2 file | Tests | Exercises | Verdict | Target in Project 3 |
|---|---:|---|---|---|
| `components/drone_control_test.cpp` | 21 (was 19, **+2** in Phase 1) | `DroneControlImpl` step/movement/scan sequencing | **Migrate ~1:1** — DI shape unchanged (§3); the +2 (`DRO08`, `DRO09` coverage) are equally mechanical, no re-scoping needed | `MissionControl/tests/component/drone_control_test.cpp` (new target, §6) |
| `components/mission_control_test.cpp` | 13 (unchanged) | `MissionControlImpl` loop/status/error/save contract | **Migrate, re-scoped** — mock one level deeper (no `IDroneControl` seam, §3) | same target as above |
| `components/mapping_algorithm_test.cpp` | 37 (unchanged) | `MappingAlgorithmImpl` internal planning logic | **Migrate ~1:1** — pImpl class, same public shape. No `ALG28`/`ALG29` test was added in Phase 1 — the supplied mutation was confirmed a no-op (§8.1) | `Algorithm/tests/component/mapping_algorithm_test.cpp` (new target, §6; needs the `RegistrationStub.cpp` + test-only `Map3DImpl`/TinyNPY link decided in Phase 3) |
| `components/mock_lidar_test.cpp` | 15 (was 14, **+1** in Phase 1) | `MockLidar` beam geometry, config, range clamping | **Migrate ~1:1** — the +1 (`LID03` coverage) is equally mechanical | `Simulator/tests/component/mock_lidar_test.cpp` (existing target extended, §6) |
| `components/scan_result_to_voxels_test.cpp` | 9 (was 7, **+2** in Phase 1) | `ScanResultToVoxels::applyToMap` voxel marking | **Migrate ~1:1** — the +2 (`DRO11`, `DRO07` coverage) are equally mechanical. No `DRO12` test was added — the supplied mutation was confirmed effectively a no-op (§8.2) | `MissionControl/tests/component/` (this utility now lives under `MissionControl`, §3) |
| `components/map3d_impl_test.cpp` | 42 (unchanged) | `Map3DImpl` bounds/config/get-set | **Migrate ~1:1** | `Simulator/tests/component/` |
| `components/maps_comparison_test.cpp` | 39 raw tests: 27 `MapsComparison::compare()` tests + 12 embedded CLI tests | `MapsComparison::compare` scoring semantics + duplicated legacy CLI coverage | **Migrate 27 near-1:1; defer 12 embedded CLI tests.** The 27 direct `MapsComparison::compare()` tests migrated in Phase 4. The remaining 12 exercise the old `drone_mapper::run()` maps-comparison CLI helper, are byte-for-byte duplicates in behavior of the separate `maps_comparison_cli_test.cpp` suite, and are deferred because Project 3 has no equivalent CLI target. | `Simulator/tests/component/maps_comparison_test.cpp` |
`NullTargetReturnsNegativeOneOnlyForThatTarget` (`COM24` coverage predates Phase 1; it was a grading-metric miss, not a real gap — §8.1) | `Simulator/tests/component/` |
| `components/maps_comparison_cli_test.cpp` | 12 (unchanged) | `maps_comparison` CLI binary behavior | **Not migrated (deferred)** — confirmed Project 3 has no `maps_comparison` CLI executable / `*_main.cpp` equivalent anywhere in `ex_3_skeleton-main`; nothing to port against yet | — (revisit if such a target is added) |
| `components/simulation_run_test.cpp` | 27 (unchanged) | `SimulationRunImpl` scoring, resolution status, error propagation | **Migrate, re-scoped** — constructor drops the `IDroneControl` argument (§3). Confirmed unchanged by Phase 1, as intended: `SIM16`/`SIM19` were correctly excluded (Project 2's `SimulationRunImpl.cpp` verified still unconditional). Those regression tests are added directly in Project 3 in Phase 5 (§10), not ported here | `Simulator/tests/component/` |
| `components/simulation_run_factory_impl_test.cpp` | 2 (unchanged) | Output-map resolution formula | **Migrate ~1:1**, thin file, no Phase 1 change — worth expanding, not just porting (low count relative to what it guards) | `Simulator/tests/component/` |
| `components/simulation_manager_test.cpp` | 15 (was 14, **+1** in Phase 1) | `SimulationManager` composition loop, per-run error handling | **Migrate, adapt** — `CompositionFilePaths`/`ReferencedConfigFile` shape changed (§3, `load_error` relocation). The +1 (`MAN27` coverage) needs the same adaptation, nothing extra | `Simulator/tests/component/` |
| `components/simulation_output_writer_test.cpp` | 1 (**new file, added in Phase 1**) | `SimulationOutputWriter`'s `min_score` summary computation | **Migrate ~1:1 (new content)** — `SimulationOutputWriter` had no component-level test at all pre-Phase-1; this one file/test closes `MAN22` (§8.3). No DI/mock dependencies, purely mechanical namespace/include port | `Simulator/tests/component/simulation_output_writer_test.cpp` (new file in this target) |
| `integration/full_flow_test.cpp` | 10 (unchanged) | End-to-end CLI + real algorithm, large "benchmark house" composition | **Migrate as-is, scenarios/assertions unchanged** — confirmed untouched by Phase 1, as required. These are among the tests that run slowly (or hit a safety-ceiling timeout) under `LID03`/`DRO08`-class mutations, but that risk is addressed by the separate focused component tests above (§9.2), not by narrowing this file's coverage. Do not shrink compositions, drop assertions, or reduce `max_steps` here. See §9.2/§9.3. | `Simulator/tests/integration/` |
| `internal/internal_flow_test.cpp` | 12 | Small bounded real-algorithm maze scenarios | **Migrate ~1:1 — validated.** Phase 1 baseline run: **12/12 pass.** These are already the "focused, bounded" style we want more of (§9.3); good template | `Simulator/tests/integration/` (or a distinct `Internal.*` filter group, matching P2's own convention of keeping this out of the graded `Integration.*` filter) |
| `audit/audit_errors_test.cpp` | 5 | Config-load failure isolation (`load_error` propagation to `-1` scoring) | **Migrate 3, adapt 1, do not migrate 1 as-is.** Phase 1 baseline run: **3/5 pass.** See §5.1 for per-test detail — two of the five need special handling before/during migration, independent of the general `ReferencedConfigFile`/`ConfigLoader` DI adaptation (§7.2) all five need | `Simulator/tests/integration/` |

**Total: 260 raw TEST/TEST_F occurrences across 15 files post-Phase-1.**
Of those, 235 raw test occurrences are migrated/planned for migration:
260 minus 12 tests in the separate deferred `maps_comparison_cli_test.cpp`,
minus the 12 duplicated embedded CLI tests inside `maps_comparison_test.cpp`,
minus 1 stale Audit test that is intentionally not migrated as-is.
The two groups of 12 CLI tests duplicate the same legacy CLI behaviors, so this
does **not** represent 24 distinct behavioral coverage areas being lost.

None of this replaces the existing 6 Project 3 "verify" tests — they test a different
concern entirely (plugin registration, cross-thread safety, `cerr` context prefixing)
and stay as-is.

### 5.1 Baseline validation results: `Internal.*` and `Audit.*`

These two suites are not part of the graded `Integration.*` filter (see
`internal/internal_flow_test.cpp`'s own comment in Project 2's `CMakeLists.txt`) and
so weren't covered by `feedback.txt`. Phase 1's baseline run against the actual
Project 2 implementation gives a clean, empirical verdict for each:

- **`Internal.*` — 12/12 pass.** No issues found. Treat all 12 as validated and
  migrate them as-is (table row above).
- **`Audit.*` — 3/5 pass.** Three tests
  (`MissingCompositionFileStillHaltsWithNoPerRunIsolationPossible`,
  `MissionBoundaryInvalidIsIsolatedToItsOwnErrorEntry`,
  `GroupLevelFailureFillsMinusOneForEveryMissionInTheGroupWhileSiblingGroupScoresNormally`)
  pass and migrate normally (subject to the general `ReferencedConfigFile` DI
  adaptation, §7.2). The other two need special handling:

  - **`Audit.NonPositiveResolutionIsIsolatedToItsOwnErrorEntry` — stale, do not
    migrate as-is.** It assumes `mission.gps_resolution_cm: 0` reaches
    `Map3DImpl` as the actual map resolution used to build the output map. The
    current implementation instead uses the *simulation's* map resolution for
    that purpose — the mission-level GPS resolution never reaches `Map3DImpl` the
    way the test expects, so the scenario it's built around cannot occur in the
    current code. **Do not modify Project 2's implementation to make this test
    pass** — the implementation is not wrong, the test's premise is outdated.
    Leave the test behind; if a genuinely reachable "non-positive/invalid
    resolution" error case exists, it would need a new test built around that
    actual case, not a repair of this one.
  - **`Audit.MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally` — valuable
    intent, stale fixture, mark for adaptation.** Its behavioral goal (one bad run
    in a batch is isolated/scored `-1` while a sibling good run in the same batch
    still scores normally) is real and worth keeping. The problem is only in how
    it currently manufactures the "bad" run: it reuses the same invalid-resolution
    scenario as the stale test above, so it fails for the same reason. **Preserve
    the test's intent during migration**, but replace the bad-run fixture with a
    genuinely reachable error case in Project 3 (e.g. a load failure or a
    mission-boundary-invalid case, both of which have their own already-passing
    Audit tests to model the fixture on) rather than porting the invalid-resolution
    trigger unchanged.

Neither finding required or should require any change to Project 2's
implementation — Project 2 stays exactly the original graded submission (§4).

---

## 6. New Test Infrastructure Needed in Project 3

Before Phase 3, only `Simulator` had a GTest-linked executable. Phase 3 has since implemented the test infrastructure described in this section; the details below remain as the design record for the targets now used in Phase 4.

1. **`MissionControl/tests/component/`** — new CMake test executable
   (e.g. `mission_control_component_test`) that compiles
   `MissionControl/src/{DroneControlImpl,MissionControlImpl,ScanResultToVoxels}.cpp`
   directly (not through the `MissionControl_322889890_315113738.so`/`.dll` plugin
   load path) + `GTest::gtest`, `GTest::gtest_main`, `GTest::gmock`, linked against
   `common::common`. This is where `drone_control_test.cpp`, `mission_control_test.cpp`,
   and `scan_result_to_voxels_test.cpp` land.
2. **`Algorithm/tests/component/`** — same pattern for
   `Algorithm/src/MappingAlgorithmImpl.cpp`, hosting `mapping_algorithm_test.cpp`.

**Decided during Phase 3 (both confirmed, both test-only, neither touches the
submitted production `.so` targets):**

- **Registration stubs.** Compiling `MappingAlgorithmImpl.cpp` /
  `MissionControlImpl.cpp` directly into a test executable still runs their
  `REGISTER_MAPPING_ALGORITHM(...)` / `REGISTER_MISSION_CONTROL(...)` macro
  invocations at static-initialization time, which would otherwise try to
  construct the real `common::MappingAlgorithmRegistration` /
  `common::MissionControlRegistration` types — pulling `Simulator`'s real
  registration/dlopen machinery into what's supposed to be an isolated component
  test. Both `MissionControl/tests/component/` and `Algorithm/tests/component/`
  therefore include a small test-only `RegistrationStub.cpp` (one per target)
  providing a stand-in `common::MappingAlgorithmRegistration` /
  `common::MissionControlRegistration` implementation that simply discards the
  factory it's handed. These stubs are compiled **only** into the test
  executables, never into `Algorithm_322889890_315113738` or
  `MissionControl_322889890_315113738` — the production plugin `.so` targets are
  unaffected and keep using the real registration path.
- **Algorithm's test-only dependency on `Map3DImpl`.** The migrated Project 2
  `mapping_algorithm_test.cpp` (37 tests) relies extensively on real map
  geometry/state — rewriting it around `GMockIMutableMap3D` throughout would
  substantially change the tests and make the more complex geometry scenarios
  hard to preserve faithfully. As an explicit, scoped exception to module
  isolation, **`algorithm_component_test` may compile/link the
  Simulator-owned `Map3DImpl.cpp` and its required test dependencies (e.g.
  TinyNPY)** for this purpose. This is a **test-only** linkage: it introduces no
  production dependency from `Algorithm` on `Simulator`, and
  `Algorithm_322889890_315113738.so` remains unchanged and independently built.
  Phase 3's own one-test smoke check for this target may continue using
  `GMockIMutableMap3D` (§6 point 5 below) — the real `Map3DImpl` link is only
  needed once the full Project 2 suite is migrated in Phase 4.
3. **`tests/mocks/`, module-local, one copy per module that needs it** — port
   `FILES PROJECT 2/tests/components/mocks/GMockI*.h` for every interface that's
   still injected: `GMockILidar`, `GMockIGPS`, `GMockIDroneMovement`,
   `GMockIMappingAlgorithm`, `GMockIMutableMap3D`, `GMockIMissionControl`. Drop
   `GMockIDroneControl`/`GMockISimulationRunFactory` if nothing in Project 3 still
   injects those interfaces from outside (confirm per §3 before deleting — worth a
   quick grep for remaining external injection points before assuming).
   **`common/` is staff-owned and must stay exactly as provided — do not add tests,
   mocks, or any student-owned helper file under it, even a shared one.** If the same
   mock (e.g. `GMockIMutableMap3D`, needed by both `Simulator` and `MissionControl`
   tests) would otherwise be duplicated verbatim across modules, prefer a small
   **non-`common` shared test-support location** instead (e.g. a top-level
   `tests_support/mocks/` or `testing/mocks/` directory outside every module, included
   only by test targets) — but default to a module-local copy unless duplication
   becomes a real maintenance problem; don't introduce a shared location speculatively.
4. **Extend the existing `Simulator` test target** (`simulator_registration_test`, or a
   new sibling `simulator_component_test` / `simulator_integration_test` if mixing
   component tests into the registration-focused binary feels wrong) with the
   `Simulator`-owned migrated files from §5's table.
5. Wire whatever `--gtest_filter=Component.*` / `--gtest_filter=Integration.*`
   convention Project 3 wants to standardize on across all three new/extended
   targets, mirroring Project 2's documented filter groups (`SimulationManager.*`,
   `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`, `MappingAlgorithm.*`,
   `MockLidar.*`, `MapsComparison.*`, `Integration.*`).

None of this touches `simulator_322889890_315113738` (the real production binary) or
the plugin `.so`/`.dll` targets — only new/extended **test** executables.

---

## 7. Migration Mechanics

### 7.1 Provenance marking

Every migrated test gets:

- A file-level banner comment at the top of each ported `.cpp`:
  ```cpp
  // Migrated from Project 2 (FILES PROJECT 2/tests/components/drone_control_test.cpp).
  // Verified against the staff mutated_src harness before porting — see
  // PROJ2_TESTS_PLAN.md §4. Adapted for Project 3's DI/module layout (see §3):
  // no other behavioral changes from the Project 2 original.
  ```
- Where a specific `TEST_F` case needed *more* than a mechanical adaptation (i.e. its
  assertions or setup meaningfully changed, not just namespaces/types), a one-line
  comment on that test explaining what changed and why, so a future reader doesn't
  assume it's a verbatim port.
- New tests written to close a coverage gap (§8) get the opposite marker:
  `// New in Project 3 — closes a Project 2 mutation-coverage gap (<BUG_ID>, see
  PROJ2_TESTS_PLAN.md §8).` so it's clear at a glance which tests are lineage vs. new.

### 7.2 Per-component adaptation notes

- **`DroneControlImpl`** (§3 confirms the DI seam is unchanged): rename includes
  (`<drone_mapper/DroneControlImpl.h>` → `<MissionControl/DroneControlImpl.h>`),
  swap `using namespace drone_mapper;` for the Project 3 namespaces
  (`common`, `common::types`, `MissionControl_322889890_315113738`), rebuild the
  `droneConfig()`/`missionConfig()`/`lidarConfig()` helper functions against
  `common::types::*`. Assertions and mock expectations carry over unchanged.
- **`MissionControlImpl`**: rebuild the fixture around `common::MissionControlDependencies`
  (a designated-initializer-friendly aggregate) instead of positional constructor
  args. Since `DroneControlImpl` is now built internally, tests that used to mock
  `IDroneControl.step()` directly must instead mock the *next layer down*
  (`ILidar`, `IGPS`, `IDroneMovement`, `IMappingAlgorithm`) and drive `runMission()`
  through a real `DroneControlImpl`. This changes several tests from pure
  interaction tests (verify a mock was called N times) into small state-based tests
  (verify the resulting `MissionRunResult` given a scripted `IMappingAlgorithm`
  mock) — note this explicitly in the "changed" comment from §7.1, since it's a
  bigger shift than a rename.
- **`SimulationRunImpl`**: drop the `IDroneControl` unique_ptr from
  `makeSimulationRun()`'s parameter list and the constructor call; everything else in
  `simulation_run_test.cpp` (map stubs, resolution-status helpers, scoring
  assertions) carries over unchanged, since `MapsComparison`, `IMutableMap3D`, and the
  constructor's remaining 7 parameters are structurally the same.
- **`SimulationManager`**: `buildSyntheticFilePaths`/`CompositionFilePaths` gained a
  `ReferencedConfigFile{path, load_error}` wrapper type (§3) in place of the old
  `std::pair<std::string, ...>`-ish shape — update fixture builders in
  `simulation_manager_test.cpp` accordingly; the load-error / group-failure
  assertions themselves are unchanged in intent.
- **Everything under `Map3DImpl`, `MockGPS`, `MockLidar`, `MockMovement`,
  `MapsComparison`, `ScanResultToVoxels`, `MappingAlgorithmImpl`**: no DI-shape
  changes were found (§3) — namespace/include updates only.
- **`MappingAlgorithmImpl`'s component-test build specifically** also needs the
  test-only `RegistrationStub.cpp` and `Map3DImpl`/TinyNPY linkage decided in §6 —
  the test file itself needs no DI-shape changes (previous bullet), but
  `algorithm_component_test`'s CMake target does need that extra wiring before
  `mapping_algorithm_test.cpp` will build there.

---

## 8. Missing Mutation Coverage Plan

`feedback.txt` bug-ID accounting (28 bugs total, cross-checked against
`mutated_src`). These are three genuinely different sets — a bug can be in more than
one — so they're kept separate rather than collapsed into a single "missing" bucket:

- **Component-caught (15):** `LID01, LID02, LID04, LID05, DRO06, DRO10, MIS14, MIS15,
  MIS17, MIS18, SIM20, MAN21, MAN23, COM25, COM26`
- **Integration-caught (8):** `LID04, LID05, DRO06, DRO07, MIS15, MAN21, MAN22, COM26`
- **Not caught by component (13 = 28 − 15):** `LID03, DRO07, DRO08, DRO09, DRO11,
  DRO12, SIM16, SIM19, MAN22, MAN27, COM24, ALG28, ALG29`
- **Not caught by integration (20 = 28 − 8):** `LID01, LID02, LID03, DRO08, DRO09,
  DRO10, DRO11, DRO12, MIS14, MIS17, MIS18, SIM16, SIM19, SIM20, MAN23, MAN27, COM24,
  COM25, ALG28, ALG29`
- **Caught by neither suite (11 = the two "not caught" sets intersected):** `LID03,
  DRO08, DRO09, DRO11, DRO12, SIM16, SIM19, MAN27, COM24, ALG28, ALG29`

`DRO07` and `MAN22` are **not** in the "caught by neither" set — both are 0% on
component but were caught by the integration suite, so they already have *some*
real detection today. They're listed in §8.3 (lower priority) rather than §8.1/§8.2,
since the gap there is "component coverage is thin" rather than "nothing detects
this at all."

Per your instruction, this section is **not** "add one test per missing ID." Each
gap below was read against its actual `mutated_src` diff to determine whether it's a
real behavioral hole worth a new test, and grouped by priority.

### 8.1 Tier 1 — real risk (hang-prone or already-confirmed bugs)

All rows below reflect the **confirmed Phase 1 outcome**, not just the pre-Phase-1
plan — see `_migration_staging/MIGRATION_MANIFEST.md` for the underlying per-file
evidence.

| Bug | Assessed gap (pre-Phase-1) | Where to test | Phase 1 outcome |
|---|---|---|---|
| `SIM16` | `mission_score` not forced to `-1` when `MissionRunStatus::Error` is returned normally (not thrown) | `Simulator` component (`SimulationRunImpl`) | **Not Project 2 Phase 1 work.** Already fixed manually in Project 3; the regression test is written directly there in Phase 5 — full spec in §10. Listed here only for cross-reference against the bug catalogue. |
| `LID03` | Nothing asserted that `MockLidar::scan()`'s `orientation` argument actually changes the beam direction (existing tests only scanned a fixed orientation toward/away from a target) | `Simulator` component (`MockLidar`) | **✅ Added** — `ScanOrientationArgumentDeterminesBeamDirection` in `mock_lidar_test.cpp` (+1). Root cause of an **integration timeout** — full analysis in §9. |
| `DRO08` | Nothing asserted `DroneControlImpl` forwards a *negative* elevate distance to `IDroneMovement::elevate()` unchanged | `MissionControl` component (`DroneControlImpl`) | **✅ Added** — `ElevateMovementForwardsNegativeDistanceUnchanged` in `drone_control_test.cpp` (+1; a one-line variant of the existing `ElevateMovementCallsElevateBeforeScan`). Also an **integration timeout** root cause — §9. |
| `ALG28` / `ALG29` | Nothing asserted `MappingAlgorithmImpl.nextStep()`'s planning actually reacts to the *content* of `latest_scan` | `Algorithm` component | **Confirmed no-op — no test added.** Phase 1 checked the supplied mutation directly in the Project 2 sandbox and confirmed it has no observable effect on `nextStep()`'s behavior (see the by-value-copy bug below) — there is no mutation to validate a test against, and the target reaction behavior wasn't judged to warrant a standalone behavioral test on its own. `mapping_algorithm_test.cpp`'s count is unchanged (37) as a result. Still open/optional if reconsidered later — not a blocker. |
| `SIM19` | Nothing asserted `result.mission_results` still contains the mission's result when `status == Error` | `Simulator` component (`SimulationRunImpl`) | **Not Project 2 Phase 1 work** (excluded on purpose, same fixture/scenario as `SIM16`) — added together with `SIM16` directly in Project 3 in Phase 5 (§10). `simulation_run_test.cpp`'s count is confirmed unchanged (27) by Phase 1. |
| `COM24` | *(pre-Phase-1 assumption)* Nothing calls `MapsComparison::compare()` with a `nullptr` entry in `targets` and asserts the documented graceful handling | `Simulator` component (`MapsComparison`) | **No action needed — the assumption was wrong.** `TEST_F(MapsComparison, NullTargetReturnsNegativeOneOnlyForThatTarget)` already existed pre-Phase-1 (confirmed: `maps_comparison_test.cpp` untouched by Phase 1, count unchanged at 39). `COM24` was a `feedback.txt` grading-time coverage-metric miss, not an actual gap in our suite — nothing was or needed to be added. |

**Confirmed during Phase 1 — `ALG28`/`ALG29`'s supplied mutation is a no-op.**
Reading `mutated_src/MappingAlgorithmImpl.cpp`:

```cpp
types::LidarScanResult& mut_scan = const_cast<types::LidarScanResult&>(*latest_scan);
for (auto hit : mut_scan) {        // by VALUE — copies each element
    hit.distance = lidar_config_.z_max;   // mutates the local copy only
}
```

Both `ALG28` and `ALG29` iterate `mut_scan` with `for (auto hit : ...)` — a
by-value copy — and then mutate the copy, not the underlying container element.
Phase 1 confirmed in the Project 2 sandbox that enabling either flag has **no
observable effect** on `nextStep()`'s behavior: the mutation itself, not our tests,
is what's broken, so no mutation-based red/green cycle is possible for these two.
Combined with the target behavior not being judged worth a standalone test on its
own merits, no new test was added for either bug.

There is also some ambiguity in the source material about whether `ALG29` is part
of the canonical 28-bug set the grading formula describes ("for each of 28 bugs")
or an additional case layered on top — this remains unresolved, and is moot given
the above.

The underlying *behavior* ("the algorithm's planning should actually depend on what
the lidar reports, not ignore it") remains a real property that *could* be worth a
future test if reconsidered — but it is not part of the current migration scope.

### 8.2 Tier 2 — real coverage gaps, no hang risk

| Bug | Assessed gap (pre-Phase-1) | Where to test | Phase 1 outcome |
|---|---|---|---|
| `DRO09` | Nothing asserted the `DroneState.step_index` seen by `IMappingAlgorithm::nextStep()` increments correctly across the *first two* `step()` calls (mutation skips incrementing on the first call only) | `MissionControl` component (`DroneControlImpl`) | **✅ Added** — `StepIndexSeenByAlgorithmIncrementsAcrossFirstTwoSteps` in `drone_control_test.cpp`. |
| `DRO11` | Nothing asserted `ScanResultToVoxels::applyToMap` is a no-op when `scan_origin` is out of the output map's bounds (the bounds check already exists in real code) | `MissionControl` component (`ScanResultToVoxels`) | **✅ Added** — `OutOfBoundsScanOriginIsANoOp` in `scan_result_to_voxels_test.cpp`. |
| `DRO12` | Nothing asserted that a *miss* only marks `Empty` voxels along the beam, never `Occupied` at the endpoint (mutation adds a spurious `Occupied` mark after a miss) | `MissionControl` component (`ScanResultToVoxels`) | **Confirmed effectively a no-op — no test added.** Phase 1 investigated the supplied mutation and found it has no reliably observable effect on stored map state for a genuine miss. Existing coverage (`MissMarksEmptyAndNeverOccupiedOrPotentiallyOccupied`, predating Phase 1) remains the relevant safety net. `scan_result_to_voxels_test.cpp`'s +2 this phase are `DRO11` and `DRO07` (§8.3), not this. Still open/optional if reconsidered later. |
| `MAN27` | Nothing asserted `score_report.score_range.min` stays `0` (per the spec's fixed `{min:0, max:100}` contract) even when the batch contains error runs | `Simulator` component (`SimulationManager`) | **✅ Added** — `ScoreRangeMinStaysZeroEvenWhenBatchContainsErrorRuns` in `simulation_manager_test.cpp`. |

### 8.3 Tier 3 — lower priority (already caught by integration, or exempted)

- `DRO07` (occupancy-priority ordering) — 0% on component pre-Phase-1, but already
  caught by the integration suite (it is in the integration-caught set in §8, so it
  is *not* one of the 11 "caught by neither" bugs), and separately exempted from the
  "missing functionality" penalty by the staff exceptions list. Optional, and
  **✅ added anyway** — `EmptyEvidenceOutranksPotentiallyOccupiedRegardlessOfWriteOrder`
  in `scan_result_to_voxels_test.cpp`, Phase 1 did this despite it not being
  required.
- `MAN22` — 0% on component pre-Phase-1 (nothing asserted `min_score` in the output
  summary reflects the true minimum of the *scored* runs when the batch also
  contains error runs), but already caught by integration. Optional, and
  **✅ added anyway** — brand-new file `simulation_output_writer_test.cpp` with
  `MinScoreReflectsTrueScoredMinimumEvenWithErrorRunsPresent` (`SimulationOutputWriter`
  had no component-level test at all before this — see the new row in §5).

### 8.4 What we're intentionally *not* adding

`LID01, LID02, LID04, LID05, DRO06, DRO10, MIS14, MIS15, MIS17, MIS18, SIM20, MAN21,
MAN23, COM25, COM26` are already caught by the existing component suite (per
`feedback.txt`) — migrate the tests that catch them (§5) and leave them alone.
`ALG28`, `ALG29`, and `DRO12` are additionally, and separately, confirmed no-op /
not worth new coverage per the Phase 1 findings above (§8.1/§8.2) — nothing more to
add there either. `COM24` needed nothing to begin with (§8.1).

---

## 9. Timeout Deep Dive: `LID03` & `DRO08`

### 9.1 Why these two hang / time out instead of failing cleanly

Both mutations were read directly in `mutated_src`:

- **`LID03`** (`MockLidar.cpp`): `scan_orientation = Orientation{};` — the requested
  scan direction is discarded and replaced with a fixed default *before* it's combined
  with the drone's GPS heading. Every scan request, regardless of what direction the
  mapping algorithm asked to look, samples the same relative cone.
- **`DRO08`** (`DroneControlImpl.cpp`): any `Elevate` command with `distance < 0` gets
  silently clamped to `0`. A commanded descent becomes a no-op; the drone's real
  height never changes even though the algorithm believes it issued a movement.

Neither mutation throws, crashes, or produces an obviously-wrong single value — both
corrupt the *feedback loop* the frontier-exploration algorithm depends on to know
what it has already explored / where it already is. The mission loop itself is
still bounded (`while (steps < mission_.max_steps)` in `MissionControlImpl`), so
this is **not a literal infinite loop** — it's that the algorithm can no longer
converge early, so *every* affected mission run burns its **entire** `max_steps`
budget instead of completing in tens/hundreds of steps.

That distinction matters for where the timeout actually surfaces: the current
`Integration.RealBenchmarkHouseFullProgramFlowAchievesFullScoreWithinTimeBudget` and
`Integration.CompositionCrossProductCoversEverySimulationMissionDroneLidarCombination`
tests (`full_flow_test.cpp`) run the **real** algorithm over a non-trivial map,
across a **cartesian product** of missions × drones × lidars. One mission silently
running to `max_steps` instead of converging is a slowdown; a whole composition of
them doing it simultaneously is what turns into an observed CI/grading timeout.
This matches the staff's own exemption list: `MIS18` and `ALG28` (both of which also
defeat convergence, per §8) are explicitly *allowed* to time out integration —
i.e., the staff already know this class of bug is a bad fit for a large end-to-end
test as the detection mechanism.

### 9.2 The strategy: add focused tests, leave the integration tests alone

**✅ Done — both focused tests below were added and validated in Phase 1** (see
§8.1's `LID03`/`DRO08` rows and the manifest). The recipe is kept here as the
rationale/spec for what was built, and as the template for `ALG28`/`ALG29`-style
cases if any are reconsidered later.

**We do not rewrite, narrow, replace, or weaken the existing integration tests to
work around this.** Their broad, real-algorithm, real-composition coverage is
valuable in its own right and stays exactly as it is (§9.3). Instead, add new
focused component tests that observe the correct behavior at a **single call**,
with no mapping loop involved at all — these become the fast, deterministic
mechanism that actually catches `LID03`/`DRO08` in normal runs, *in addition to*
the existing integration tests, not instead of them:

- `LID03` → call `MockLidar::scan(orientation)` twice with two different orientations
  against a map with an obstacle in only one of those directions; assert the two
  results differ (one hit, one miss). Pure function call, microseconds, cannot hang.
- `DRO08` → drive `DroneControlImpl::step()` once with a mocked `IMappingAlgorithm`
  that returns a single `Elevate(-10cm)` command; assert
  `EXPECT_CALL(movement_, elevate(-10.0 * isq::length[cm]))` receives the negative
  value unchanged (Project 2's `drone_control_test.cpp` already has
  `ElevateMovementCallsElevateBeforeScan` as a template — this is that test with a
  negative distance). One `step()` call, no loop, cannot hang.

Neither of these requires running the real `MappingAlgorithmImpl` or a real
multi-mission composition. This is the general principle to apply to `ALG28`/`ALG29`
too (§8.1): test the narrow contract the mutation breaks, not the emergent behavior
of the full search algorithm — as a new, additional test, not a substitute for
existing coverage.

### 9.3 The existing integration tests are not touched

`full_flow_test.cpp`'s existing tests
(`RealBenchmarkHouseFullProgramFlowAchievesFullScoreWithinTimeBudget`,
`CompositionCrossProductCoversEverySimulationMissionDroneLidarCombination`, etc.)
keep their current scenarios, compositions, `max_steps`, and assertions completely
unchanged, both in Project 2 and when migrated to Project 3 (§5). Specifically:

- **Do not** shrink `max_steps`, reduce the cartesian product, or drop any existing
  assertion (including any elapsed-time assertion) in these tests to make them
  faster or to dodge the `LID03`/`DRO08` timeout.
- It is expected and acceptable that these tests may still run slowly, or take a
  long time, under `LID03`/`DRO08` (or a similar convergence-defeating bug) during
  one-time mutation validation (§9.4) — that is not a defect in the test suite, and
  is not, by itself, a reason to reduce their behavioral coverage. The point of
  §9.2's focused tests is that day-to-day CI never has to wait for that to happen
  to get a signal.
- The **only** thing that may be added around these tests, in Project 3, is an
  external `ctest`/build-system safety ceiling —
  `set_tests_properties(<test> PROPERTIES TIMEOUT <n>)` or equivalent — set
  generously enough not to flake on a legitimately slow-but-correct run. This
  ceiling exists purely so a genuine hang fails the CI run within a bounded time
  instead of hanging the whole test binary; it is a backstop, not a detection
  mechanism, and it does not replace or substitute for any assertion the test
  already makes.

### 9.4 Validation sequence before migrating these two focused tests

Follow §4.1 exactly, with one extra check specific to these two bugs:

1. Reproduce `LID03` (resp. `DRO08`) in the Project 2 sandbox.
2. Run the **new focused test** (not the full integration suite) — confirm it fails
   fast (should take milliseconds, proving it doesn't rely on the mapping loop).
3. Separately, run the existing large integration test
   (`RealBenchmarkHouseFullProgramFlowAchievesFullScoreWithinTimeBudget` or similar),
   **scenario and assertions unchanged**, under the same mutation with a wall-clock
   cap on the invocation itself (e.g. a shell/CI-level timeout of ~30–60s — not a
   change to the test) purely to *confirm* the slowdown/timeout reproduces as
   expected. This is a one-time sanity check; per §9.3, it is fine for this to be
   slow or to hit the cap — that outcome is not a reason to alter the test.
4. Flip the flag off, confirm both the focused test and the integration test pass
   normally and quickly again.
5. Migrate **both** the focused test and the (unchanged) integration test into
   Project 3 (§7) — the focused test is additive, not a replacement. Add the
   external `ctest` safety ceiling from §9.3 to the integration test at that point,
   if one doesn't already exist.

---

## 10. Regression Test Plan: `MissionRunStatus::Error → mission_score == -1`

This protects the fix for the confirmed carried-over `SIM16` bug in
`SimulationRunImpl::run()`. **The fix has already been applied manually in Project 3**
(not part of this document — that was a separate, already-completed change). This
entire section is **Project 3-only work**: it is not run through the Project 2
validate-first workflow in §4, and Project 2's source is not touched for it — see
§4's `SIM16` carve-out for why.

### 10.1 Primary assertion

New `TEST_F(SimulationRun, …)` in the migrated `simulation_run_test.cpp`
(`Simulator/tests/component/`):

- Script the `mission_control_` mock (`GMockIMissionControl`, per the existing
  `makeSimulationRun()` fixture pattern) to return a `MissionRunResult` with
  `status == MissionRunStatus::Error` (no exception thrown — this must be the
  **normal-return** path, not `RunPropagatesExceptionFromMissionControlRatherThan
  SwallowingIt`'s throw path, since that's a different, already-tested code path).
  Reuse the `RunReturnsMissionResultFromMissionControl` test at line 255 of the
  Project 2 original as the direct template — same shape, different status value.
- Assert `result.mission_score == -1.0` (`EXPECT_DOUBLE_EQ`).
- Assert `result.mission_results` still contains the errored `MissionRunResult`
  unchanged (this doubles as the `SIM19` coverage-gap test from §8.1 — same
  fixture, same scenario, no reason to duplicate it as a separate test).

### 10.2 "Map comparison not required for an errored mission"

This **is** testable today, confirmed by reading the existing fixture
(`makeStubMap()` in `simulation_run_test.cpp` — `hidden_map`/`output_map` are
`NiceMock<GMockIMutableMap3D>` doubles, not real `Map3DImpl` instances). The
existing file's own comment already documents that `MapsComparison::compare()` is "a
hardwired collaborator, not injected through an interface" and is *currently*
**always** called — so this assertion will fail against today's code and should
only be added once the fix restructures `run()` to short-circuit before calling
`MapsComparison::compare()` on the error path (not call it and discard the result
afterward — the whole point is avoiding the comparison work, and it's also the only
way this test can mean anything).

**`getMapConfig()` is not a valid proxy for this — do not use it.**
`SimulationRunImpl::run()` calls `hidden_map_->getMapConfig()` and
`output_map_->getMapConfig()` directly and unconditionally, independent of
`MapsComparison` entirely (confirmed by reading the implementation: in both
Project 2's and Project 3's `SimulationRunImpl.cpp`, `hidden_map_->getMapConfig()`
is called near the top of `run()` before the error check even happens, and
`output_map_->getMapConfig()` is called on both the early-return and normal paths
to populate `result.output_map_config`). An `EXPECT_CALL(..., getMapConfig())
.Times(0)` would fail on the *happy* path too, so it proves nothing about whether
comparison specifically ran.

`atVoxel()` is the correct, comparison-specific proxy — confirmed by reading both
`MapsComparison::compare()`'s implementation (`src/MapsComparison.cpp`: it calls
`origin.atVoxel(world_pos)` / `target.atVoxel(world_pos)` per sampled voxel — this
holds identically in Project 2's and Project 3's `MapsComparison.cpp`) and
`SimulationRunImpl::run()`'s implementation (it never calls `atVoxel()` on either
map itself, directly or indirectly, outside of the `MapsComparison::compare()`
call). So once the fix's short-circuit exists:

- `EXPECT_CALL(*hidden_map_mock, atVoxel(_)).Times(0);` (and the same on
  `output_map_mock`) on the error path — this is genuinely specific to
  `MapsComparison::compare()` never having run, without needing to mock
  `MapsComparison` itself (it has no interface to mock; this indirect assertion on
  its collaborators is the available seam).
- Pair with a second test confirming the **non-error** path still calls into the
  stub maps as before (regression-proofing the happy path isn't accidentally
  short-circuited too) — several existing tests
  (`RunComputesNonNegativeScoreOnSuccess`, `RunScoresFromTheRealMapsComparison...`)
  already cover this implicitly; just confirm they still pass unchanged.

### 10.3 Validation sequence

The §4.1 protocol (Project 2 sandbox, `mutated_src` flags) does not apply here —
see §4's `SIM16` carve-out. This is validated entirely inside Project 3, and the
fix is already in place there, so the sequence runs "backwards" relative to a
normal red-first TDD flow:

1. Write the test against **current** Project 3 `SimulationRunImpl::run()` (fix
   already applied) — confirm it **passes**.
2. As a one-time sanity check that the test actually exercises the fixed code path
   (not a vacuously-true assertion), temporarily revert just the fix locally —
   comment out the status check / restore the unconditional `scores[0]` assignment
   — rebuild, and confirm the test **fails**. This is a local, throwaway edit for
   verification only; it is never committed.
3. Restore the fix, rebuild, confirm green again, and confirm no other
   `SimulationRun.*` test regressed.

---

## 11. Suggested Order of Execution

1. **✅ Done (Phase 1).** Open the existing disposable Project 2 copy and establish
   the clean test baseline.
2. **✅ Done (Phase 3).** Stand up the new CMake test targets in Project 3 (§6) with
   just the migrated mock headers and **one** trivial ported test each, to prove the
   build/link/DI plumbing works before porting everything. Includes the
   `RegistrationStub.cpp` and test-only `Map3DImpl`/TinyNPY decisions (§6).
3. **→ Next (Phase 4).** Migrate the low-risk, high-volume files first
   (`map3d_impl_test.cpp`, `maps_comparison_test.cpp`, `mock_lidar_test.cpp`,
   `scan_result_to_voxels_test.cpp`, `drone_control_test.cpp` — DI seam intact, §3,
   so it belongs here and *not* with the re-scoped files in step 4,
   `simulation_run_factory_impl_test.cpp`, `simulation_output_writer_test.cpp` — new
   Phase 1 content, equally mechanical) — mechanical, builds confidence in the port
   process. Migrate `mapping_algorithm_test.cpp` alongside these once
   `algorithm_component_test`'s test-only `Map3DImpl`/TinyNPY link and
   `RegistrationStub.cpp` (§6) are wired up — the test file itself is equally
   mechanical, only the target's build needs the extra step first.
4. **→ Next (Phase 4).** Migrate the re-scoped files (`mission_control_test.cpp`,
   `simulation_run_test.cpp`, `simulation_manager_test.cpp`) — needs the §7.2 DI
   adaptations. (`drone_control_test.cpp` moved to step 3 — its DI seam is
   unchanged, it was never actually structurally adapted.)
5. **→ Next (Phase 5) — only `SIM16`/`SIM19` remain here.** Every other Tier 1 gap
   (§8.1: `LID03`, `DRO08`, `ALG28`/`ALG29`, `COM24`) is already resolved as of
   Phase 1 — added, confirmed no-op, or confirmed already covered, respectively.
   `SIM16`/`SIM19` are the only Tier 1 items still to write, and they're written
   directly in Project 3 (§10), not ported.
6. **→ Next (Phase 6).** Migrate the integration/internal/audit suites (§5's last
   three rows), unchanged in scenario/assertions (§9.3) — the only addition is the
   optional external `ctest` safety-ceiling timeout, not a scenario change. Apply
   the `Audit.*` exceptions from §5.1 (skip the stale resolution test, adapt the
   mixed-batch test's bad-run fixture).
7. **✅ Done (Phase 1) — nothing left to add.** Tier 2 (§8.2: `DRO09`, `DRO11`,
   `MAN27`) and Tier 3 (§8.3: `DRO07`, `MAN22`) are all added, except `DRO12`
   (§8.2), which was confirmed effectively a no-op and intentionally left without a
   new test.
8. **→ Next (Phase 6).** Full-suite run in Project 3, both new targets + extended
   `Simulator` target, confirm nothing from the original 6 "verify" tests
   regressed.
