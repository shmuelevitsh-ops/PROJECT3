# Project 2 → Project 3 Test Migration — Execution Plan

Sequential, agent-facing checklist. For rationale, mutation analysis, architecture
deltas, and detailed per-bug reasoning, see `PROJ2_TESTS_PLAN.md` — this document
only says what to do and in what order.

**Container constraint:** Project 2 and Project 3 live in separate Dev Containers.
An agent working in one can only see that one. Phases 1–2 run inside the **Project 2**
container; Phases 3–6 run inside the **Project 3** container. Phase 2 is the handoff:
it physically copies files from the Project 2 tree into the Project 3 tree so they
exist and are visible once the Project 3 container is opened.

`project 3\FILES PROJECT 2` is already a disposable copy — edit it freely during
validation. `EXSTRA FILES\mutated_src` is a disposable staff-supplied copy — use it
freely too. No extra sandbox copy is needed.

**Progress: Phases 1–4 are ✅ complete. Phase 5 is next.**
Phase 4 migrated and validated all in-scope component tests. The only deferred
component coverage is the legacy maps-comparison CLI coverage, for which Project 3
has no equivalent CLI target. Phase 5 now adds the Project-3-specific SIM16/SIM19
regression coverage.

---

## PHASE 1 — Project 2 validation and repair ✅ COMPLETE

**Open:** `project 3\FILES PROJECT 2` in the Project 2 Dev Container.

**Tasks:**
1. Build `drone_mapper_simulation_test` and run the full `Component.*`/`Integration.*`
   suite. Also build and run `Internal.*` and `Audit.*` — these are not part of the
   default target glob (local-dev-only suites, excluded from the graded submission
   archive), so extend the glob locally / add a small ad-hoc target to run them.
   Record the baseline. **Confirmed baseline (already validated, carry forward as
   ground truth — see `PROJ2_TESTS_PLAN.md` §5.1 for detail):** `Internal.*` 12/12
   pass; `Audit.*` 3/5 pass, with two known exceptions (next bullet).
   - `Audit.NonPositiveResolutionIsIsolatedToItsOwnErrorEntry` is **stale**: it
     assumes `mission.gps_resolution_cm: 0` reaches `Map3DImpl` as the map
     resolution, but the implementation actually uses the simulation's map
     resolution instead. **Do not modify Project 2's implementation to make this
     pass.** Do not migrate it as-is (Phase 2 marks it "not migrated").
   - `Audit.MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally` has a valuable
     intent (bad run isolated/scored `-1`, sibling good run in the same batch still
     scores normally) but currently manufactures its "bad" run via the same stale
     invalid-resolution scenario, so it fails for the same reason. Preserve the
     intent; flag it in the Phase 2 manifest as "structurally adapted" — its
     bad-run fixture gets replaced with a genuinely reachable error case (e.g. a
     load failure or mission-boundary-invalid case, per already-passing sibling
     Audit tests) during migration, not repaired here.
2. Review each remaining existing test file for continued validity (see
   `PROJ2_TESTS_PLAN.md` §5 for the full inventory) — keep everything still valid,
   don't rewrite tests that already pass and still test something real.
3. Add/repair focused tests for the Tier 1 + Tier 2 coverage gaps in
   `PROJ2_TESTS_PLAN.md` §8.1/§8.2. Skip §8.3 unless time allows.
   **Exception: skip `SIM16` and `SIM19` entirely in this phase.** `SIM16`'s
   mutation flag is inverted relative to every other bug (its Project 2 baseline
   already has the bug unconditionally; the flag makes it *correct*, not broken),
   so validating it here would mean patching Project 2's real implementation —
   which this phase must not do. `SIM16` is already fixed manually in Project 3;
   both it and the adjacent `SIM19` gap are added directly in Project 3 during
   Phase 5 instead (see `PROJ2_TESTS_PLAN.md` §4's `SIM16` carve-out and §10).
4. Specifically for `LID03` and `DRO08`: add the narrow, single-call component
   tests described in `PROJ2_TESTS_PLAN.md` §9.2 (`MockLidar::scan()` orientation
   sensitivity; `DroneControlImpl::step()` forwarding a negative elevate distance
   unchanged). These must run in milliseconds and must not depend on the real
   mapping loop or a large composition.
5. For `ALG28`/`ALG29`: before relying on the supplied mutation flags, check in this
   sandbox whether enabling them actually changes observable behavior (see
   `PROJ2_TESTS_PLAN.md` §8.1 caveat — the mutation may be a no-op due to a
   by-value copy). Write the behavioral test either way if the underlying property
   ("planning reacts to real scan content") is worth protecting; don't block on the
   mutation flag working.
6. Validate every new/repaired test against `mutated_src` where the mutation is
   trustworthy, using the sandbox mechanics in `PROJ2_TESTS_PLAN.md` §4.2:
   - normal Project 2 build → target test passes;
   - rebuild with the one relevant `-D<BUG_ID>` → target test fails, and fails
     *quickly* (no waiting for a timeout);
   - flag removed, rebuild → test passes again.
7. For the two timeout bugs specifically, do a one-time sanity check per
   `PROJ2_TESTS_PLAN.md` §9.4 step 3: confirm the *old* large integration test
   still exhibits the slow/hang behavior under the mutation (short wall-clock cap
   on the run, just to confirm — not something to keep relying on).

**Do not touch:** anything outside `FILES PROJECT 2`. This phase never opens or
edits `ex_3_skeleton-main`.

**Exit condition:** every test intended for migration (full list finalized in
Phase 2) passes on a clean Project 2 build — with the two named `Audit.*`
exceptions handled as above (one excluded, one flagged for fixture adaptation, not
required to pass as-is). Every Tier 1 test *except* `SIM16`/`SIM19` (explicitly out
of scope here, see above) demonstrably fails under its corresponding mutation flag
and passes again once removed — for `ALG28`/`ALG29`, this holds only if step 5's
check found the flag has an observable effect at all; otherwise record that finding
and rely on the behavioral test alone. The `LID03`/`DRO08` focused tests fail fast
under mutation (milliseconds); it's fine if the *existing* integration test used
for the one-time sanity check in step 7 is slow or hits its wall-clock cap under
the same mutation — that is expected, not a problem to fix.

---

## PHASE 2 — Prepare the migration handoff ✅ COMPLETE

**Still in:** Project 2 container (or either — this is file bookkeeping, not code
that needs to build).

**Tasks:**
1. Finalize the set of validated Project 2 tests to migrate, using
   `PROJ2_TESTS_PLAN.md` §5's table as the starting point, updated with anything
   added/repaired in Phase 1.
2. Write a short **migration manifest** (new file,
   `FILES PROJECT 2/tests/MIGRATION_MANIFEST.md` or similar) listing, per source
   test file: its Project 3 destination path, and one of:
   - **near-1:1** — mechanical port only (namespaces/includes),
   - **structurally adapted** — DI/constructor shape changed, needs rework
     (`mission_control_test.cpp`, `simulation_run_test.cpp`,
     `simulation_manager_test.cpp` — see `PROJ2_TESTS_PLAN.md` §7.2),
   - **new, added from feedback gaps** — the Phase 1 additions,
   - **not migrated** — plus a one-line reason (e.g. `maps_comparison_cli_test.cpp`
     if no equivalent CLI target exists in Project 3 — confirm before excluding).
   Most files get one verdict; `audit_errors_test.cpp` needs per-test-case entries
   instead of one for the whole file: 3 of its 5 tests are near-1:1 (modulo the
   general `ReferencedConfigFile` adaptation, §7.2),
   `NonPositiveResolutionIsIsolatedToItsOwnErrorEntry` is **not migrated** (stale,
   §5.1), and `MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally` is
   **structurally adapted** (bad-run fixture replaced, §5.1) — list all three
   outcomes for this one file rather than a single verdict.
3. Physically copy the validated files into the Project 3 tree so they're visible
   once that container is opened:
   - **Copy:** the test `.cpp` files being migrated, their GMock mock headers
     (`tests/components/mocks/GMockI*.h`), and any test-input fixtures they need
     (YAML configs, `.npy` maps under `tests/integration/test_inputs/`,
     `tests/audit/audit_inputs/`).
   - **Destination:** stage them under a clearly temporary location in the Project 3
     tree, e.g. `ex_3_skeleton-main/_migration_staging/<module>/...`, mirroring the
     Phase 3 destinations from `PROJ2_TESTS_PLAN.md` §5/§6 — this keeps Phase 4's
     "adapt in place" work from being confused with real, already-wired test
     targets before the plumbing exists.
   - **Do not copy:** `EXSTRA FILES/mutated_src` (staff disposable copy, stays in
     Project 2 only — never let `#ifdef <BUG>` blocks reach Project 3 source),
     `FILES PROJECT 2/src` or `include` (Project 3 has its own implementation),
     anything under `FILES PROJECT 2/build` or generated output
     (`tests/audit/audit_output/**` results, CMake cache), and the `.docx`/`.txt`
     staff feedback material (already fully digested into `PROJ2_TESTS_PLAN.md`).

**Do not touch:** any file under `ex_3_skeleton-main` other than adding the new
staging copies — no edits to existing Project 3 source or tests yet.

**Exit condition:** the manifest is complete and accounted for against every file
in `PROJ2_TESTS_PLAN.md` §5, and every file the manifest marks for migration exists
under `ex_3_skeleton-main/_migration_staging/`.

---

## PHASE 3 — Project 3 test infrastructure ✅ COMPLETE

**Open:** `project 3\ex_3_skeleton-main` in the Project 3 Dev Container.

**Tasks:**
1. Create/adjust CMake test targets per `PROJ2_TESTS_PLAN.md` §6:
   - `MissionControl/tests/component/` — new test executable compiling
     `MissionControl/src/*.cpp` directly, GTest+GMock linked.
   - `Algorithm/tests/component/` — same, for `Algorithm/src/MappingAlgorithmImpl.cpp`.
   - `Simulator` — extend the existing test target (or add a sibling) to host the
     `Simulator`-owned migrated files.
   - **Decided this phase (recorded in `PROJ2_TESTS_PLAN.md` §6):** both the
     `MissionControl` and `Algorithm` test targets need a test-only
     `RegistrationStub.cpp` providing a stand-in `common::MissionControlRegistration`
     / `common::MappingAlgorithmRegistration` that discards the factory — otherwise
     compiling `MissionControlImpl.cpp`/`MappingAlgorithmImpl.cpp` directly into the
     test binary triggers their `REGISTER_...` macro at static init and pulls in
     `Simulator`'s real registration/dlopen machinery. These stubs go **only** in
     the test executables — never in `MissionControl_322889890_315113738` or
     `Algorithm_322889890_315113738`.
   - **Also decided:** `algorithm_component_test` is permitted to compile/link the
     Simulator-owned `Map3DImpl.cpp` (+ TinyNPY) as a **test-only** dependency, so
     Project 2's `mapping_algorithm_test.cpp` (which relies extensively on real map
     geometry) can migrate largely as-is in Phase 4 instead of being rewritten
     around `GMockIMutableMap3D`. This is not needed for this phase's smoke test
     (next bullet) — only once the full suite migrates. It introduces no
     production dependency from `Algorithm` on `Simulator`;
     `Algorithm_322889890_315113738.so` stays unchanged and independently built.
2. Port the GMock mock headers into module-local `tests/mocks/` directories (one
   copy per module that needs it; a shared non-`common` location only if
   duplication becomes a real problem — see `PROJ2_TESTS_PLAN.md` §6 point 3).
3. **Do not modify anything under `common/`.** It is staff-owned and must stay
   exactly as provided — no tests, no mocks, no student helper files there, shared
   or otherwise.
4. Keep all 6 existing `Simulator/tests/*_verify*` / `registration` tests
   untouched and passing.
5. Prove the plumbing before mass migration: pick **one** trivial test per new/
   extended target (e.g. one `DroneControlImpl` test, one `MappingAlgorithmImpl`
   test, one `Simulator`-side test) from the staging area, adapt it, wire it in,
   and get it compiling, linking, and passing. The `Algorithm` smoke test may use
   `GMockIMutableMap3D` for this one-test check — it does not need the real
   `Map3DImpl` link from task 1 yet; that's only required once the full
   `mapping_algorithm_test.cpp` suite migrates in Phase 4.

**Do not touch:** `common/`. Do not yet bulk-copy the rest of
`_migration_staging/` into real targets — that's Phase 4. The `RegistrationStub.cpp`
files and the `Algorithm`→`Map3DImpl` test-only link belong strictly to the new
test executables — do not let either touch `Algorithm_322889890_315113738` or
`MissionControl_322889890_315113738`'s own CMake target.

**Exit condition:** the one smoke test per module compiles, links, runs, and is
discovered by the test runner (`--gtest_list_tests` shows it) — for all three
targets (`Simulator`, `MissionControl`, `Algorithm`).

---

## PHASE 4 — Adapt and migrate validated P2 tests ✅ COMPLETE

**Environment:** Project 3 container (continued).

**Tasks:**
1. Migrate low-risk/mechanical files first (per manifest "near-1:1" entries):
   `map3d_impl_test.cpp`, `maps_comparison_test.cpp`, `mock_lidar_test.cpp`,
   `scan_result_to_voxels_test.cpp`, `drone_control_test.cpp` (its `ILidar`/`IGPS`/
   `IDroneMovement`/`IMappingAlgorithm`/`IMutableMap3D` DI seam is intact per §3/§7.2
   — it only needs the standard namespace/include port, **not** the re-scoping
   step 3 below applies to `MissionControlImpl` — don't group it with those),
   `simulation_run_factory_impl_test.cpp` (thin, no Phase 1 change),
   `simulation_output_writer_test.cpp` (new Phase 1 file, no mocks/DI — see the
   manifest's `simulation_output_writer_test.cpp` row).
2. Migrate `mapping_algorithm_test.cpp` (37 tests). The test file itself needs no
   DI-shape changes (`PROJ2_TESTS_PLAN.md` §7.2), but before/while porting it,
   wire `algorithm_component_test`'s CMake target with the test-only
   `Map3DImpl.cpp`/TinyNPY link and `RegistrationStub.cpp` decided in Phase 3
   (`PROJ2_TESTS_PLAN.md` §6) — this is what lets the suite keep using real map
   geometry instead of being rewritten around `GMockIMutableMap3D`.
3. Then migrate the structurally-adapted suites, applying the specific DI changes
   in `PROJ2_TESTS_PLAN.md` §7.2: `mission_control_test.cpp`
   (re-scoped — no more direct `IDroneControl` injection seam),
   `simulation_run_test.cpp` (constructor drops the `IDroneControl` argument),
   `simulation_manager_test.cpp` (`ReferencedConfigFile` shape). This is the
   complete list — `drone_control_test.cpp` is **not** one of these (see step 1).
4. For every migrated test: preserve intent, inputs, and assertions as close to
   1:1 as reasonable; change only what the new architecture forces (namespaces,
   includes, constructor/DI shape).
5. Mark every migrated file/test per `PROJ2_TESTS_PLAN.md` §7.1's provenance
   convention (banner comment + per-test note where behavior, not just mechanics,
   changed).
6. Delete the corresponding files from `_migration_staging/` once ported (don't
   leave stale duplicates around).

**Do not touch:** `common/`. Do not modify Project 3's production `src/` to make a
test pass — if a migrated test fails against current Project 3 behavior, stop and
flag it rather than silently adjusting the assertion or patching production code
under a test-migration task. (`SIM16` is already fixed in Project 3 prior to this
phase, so no migrated test should be failing because of it — its regression test is
added separately in Phase 5.)

**Exit condition:** all tests migrated in this phase build and pass against
current (unmodified) Project 3 code.

---

## PHASE 5 — Project 3-specific regression coverage ✅ COMPLETE

**Environment:** Project 3 container (continued).

**Tasks:**
1. Add the `SIM16` regression test in the migrated `simulation_run_test.cpp`
   (`PROJ2_TESTS_PLAN.md` §10.1): `MissionRunStatus::Error` returned normally from
   `mission_control_` → `result.mission_score == -1.0`, and `result.mission_results`
   still contains that result (this also covers the `SIM19` coverage gap — one
   test, don't duplicate). **This is entirely new work done here, not a Project 2
   port** — `SIM16` was explicitly excluded from Phase 1 (see Phase 1's exception
   note) precisely so it could be written directly against Project 3.
2. The fix (`mission_score == -1.0` on error) is **already applied manually** in
   Project 3 — write the test against current code and confirm it **passes**. Then,
   as a one-time local sanity check that the test isn't vacuous, temporarily revert
   just the fix (restore the unconditional `scores[0]` assignment), rebuild, confirm
   the test **fails**, then restore the fix and confirm green again. This revert is
   throwaway/local only — never commit it.
3. Check whether the applied fix also short-circuits `MapsComparison::compare()`
   entirely on the error path (as opposed to still computing `scores[0]` and only
   conditionally assigning it). If it does, add the "comparison was skipped"
   assertion using `EXPECT_CALL(*hidden_map_mock, atVoxel(_)).Times(0)` (and the
   same on `output_map_mock`) — **not** `getMapConfig()`, which is called
   unconditionally elsewhere in `run()` and proves nothing (see
   `PROJ2_TESTS_PLAN.md` §10.2 for why, confirmed against both
   `SimulationRunImpl.cpp` and `MapsComparison.cpp`). If the fix does not
   short-circuit the call, skip this assertion rather than forcing an
   implementation change here — this phase adds regression tests, it does not
   redesign the already-applied fix.
4. Add a companion test confirming the non-error path is unaffected (still calls
   into the maps as before) — several existing migrated tests already cover this
   implicitly; just confirm they still pass.
5. Fold in any Tier 1/2 gap tests from Phase 1 that needed real rework during
   porting (not just mechanical adaptation) — call these out explicitly as
   Project-3-specific, per the provenance convention.

**Exit condition:** the `SIM16`/`SIM19` regression tests exist and pass against the
already-applied fix; the throwaway local revert confirmed they fail without it; and
if a "comparison skipped" assertion was added, it uses `atVoxel()`, not
`getMapConfig()`.

---

## PHASE 6 — Integration and final regression ✅ COMPLETE

**Environment:** Project 3 container (continued).

**Tasks:**
1. Migrate `internal_flow_test.cpp` (12 tests, all validated in Phase 1) and
   `audit_errors_test.cpp` per its Phase 1/§5.1 breakdown: 3 tests near-1:1, skip
   `NonPositiveResolutionIsIsolatedToItsOwnErrorEntry` (stale, not migrated),
   rebuild `MixedBatchIsolatesBadRunsWhileScoringTheGoodRunNormally`'s bad-run
   fixture around a genuinely reachable error case while keeping its isolation
   assertion intent. Apply §7.2's `SimulationManager`/`ConfigLoader` adaptation
   notes throughout.
2. Migrate `full_flow_test.cpp` **with its scenarios, compositions, `max_steps`,
   and assertions unchanged** — do not narrow, shrink, or otherwise rewrite these
   tests because of the `LID03`/`DRO08` timeout risk (`PROJ2_TESTS_PLAN.md` §9.3);
   that risk is already covered by the focused component tests added in Phase 1/5.
   The only permitted addition is an external `ctest` `TIMEOUT` property (or
   equivalent) as a safety ceiling, generous enough not to flake a legitimately
   slow-but-correct run — it's a backstop, not a scenario change.
3. Run the full test suite: all migrated tests + all pre-existing Project 3 tests
   (the 6 verify/registration tests) together, across all three targets.
4. Confirm `_migration_staging/` is empty and can be deleted.

**DONE when:**
- Every test in the final migration manifest is either migrated, or marked "not
  migrated" with a recorded reason.
- The full Project 3 test suite (existing + migrated + new regression tests)
  passes.
- `LID03`/`DRO08`-class bugs are caught, in normal day-to-day runs, by the fast
  focused component tests — not by waiting for `full_flow_test.cpp` to time out.
  `full_flow_test.cpp`'s own scenarios and assertions are unchanged from Project 2;
  any `ctest` timeout on it is a safety ceiling only.
- `common/` is unchanged from its staff-provided state.
- `_migration_staging/` no longer exists.
