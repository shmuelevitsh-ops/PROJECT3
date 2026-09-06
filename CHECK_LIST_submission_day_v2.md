# Assignment 3 — Submission Checklist

This checklist is intentionally split into **three stages**:

- **PART A — Final whole-project code scan:** grading/code-quality issues, critical safety issues, comment cleanup, and Known Issues preparation.
- **PART C — Whole-project functional/architectural audits:** namespaces, targets, project boundaries, registration, `.so` lifecycle, threading, CLI, and outputs.
- **PART D — Final pre-submission audit:** build, execution, diagnostics, README, and packaging.

## HARD SCOPE

PART A applies **only** inside `ex_3_skeleton-main`, and only to student-owned production code under:

- `Algorithm/`
- `MissionControl/`
- `Simulator/`
- `UserCommon/`

Do not inspect or modify other copies of the project.

## HARD EXCLUSION — TESTS / STAFF CODE

Student tests are **not part of the submission**.

During PART A and PART C:
- Do **not** review, fix, refactor, or clean comments in tests.
- Do **not** modify test fixtures/support files.
- Do **not** report test-only issues.
- Do **not** add test-only findings to Known Issues.
- Do **not** modify staff-provided `common/` or other staff-owned interface code.

Only inspect test-related build configuration if it accidentally affects the submitted production build/package.

## Source priority

If anything conflicts:

1. Current Assignment 3 instructions.
2. Current course skeleton and staff-provided interfaces.
3. Assignment 2 rules that Assignment 3 did not redefine.
4. Course grading/error-code guidance and official Known Issues categories.

---

# PART A — FINAL WHOLE-PROJECT CODE SCAN

Run once over all student-owned production code in the scope above.

This is **not** a learning review.

**Important:** every genuine issue matching the staff grading criteria must be reported, even if:
- the program currently works;
- the issue would require refactoring;
- we later decide not to fix it before submission.

Do not silently filter grading issues as “nice to have”.  
First identify them; only afterwards decide whether they are worth fixing now.

## A1. Staff grading/code-quality criteria

Check for:

- [ ] `e01/e02` — bad header/class organization.
- [ ] `e03/e16` — incorrect `mp-units` use or replacing strong project types with plain numerics.
- [ ] `e04` — incorrect standard-library/container/algorithm usage.
- [ ] `e05` — diagnostics handled/passed at the wrong layer.
- [ ] `e06/e07` — unnecessary copies/mutable references or missing `const`.
- [ ] `e08/e22` — unnecessary public API or poor encapsulation.
- [ ] `e09` — long functions.
- [ ] `e10` — duplicated classes, functions, or **logical flows**, including non-identical duplication across files/classes.
- [ ] `e11/e17` — unapproved/manual dependency or unnecessary dependency/include.
- [ ] `e13` — incorrect pointer use.
- [ ] `e21` — repeated expensive computation.
- [ ] `e23` — meaningful hard-coded/magic values.

Also check the official Known Issues Coding categories, including:
- [ ] missing/redundant class;
- [ ] code duplication;
- [ ] function too long;
- [ ] class doing another class's job;
- [ ] spread logic;
- [ ] code that is unnecessarily complex;
- [ ] hard-coded values instead of constants/enums;
- [ ] materially bad names.

### Mandatory visibility for e09/e10

- [ ] Identify **all production functions around 40+ lines**, then judge whether each is genuinely problematic or justified.
- [ ] Actively compare similar flows across the project, especially constructors, parsing/validation/error handling, and comparative-vs-competition orchestration.
- [ ] Do not restrict duplication detection to literal copy-paste.

## A2. Critical correctness/safety

Also flag clear submission risks involving:

- [ ] dangling pointers/references/lambda captures;
- [ ] invalid ownership/resource lifetime or leaks;
- [ ] null dereference or clear undefined behavior;
- [ ] data races or plausible deadlocks;
- [ ] exceptions escaping worker-thread functions;
- [ ] unsafe manual lock/unlock;
- [ ] invalid `.so` lifetime, especially `dlclose` while an instance/factory/callback/deleter from the library is still alive.

Broader threading and `.so` architecture are still checked in PART C.

## A3. Comment cleanup — perform during this pass

Aggressively remove redundant comments from student-owned production code.

Keep comments only when they explain non-obvious:
- intent;
- assignment-specific behavior;
- lifetime/ownership;
- synchronization/concurrency;
- implementation decisions.

Remove comments that merely restate code, explain basic C++, narrate trivial flow, contain stale TODOs, or preserve commented-out implementation.

Comment cleanup must not change behavior.

## A4. Known Issues workbook — working audit output

During PART A, use the official Assignment-3 Known Issues workbook as the **complete working record of the defects found**.

- [ ] Preserve the official workbook structure and allowed values.
- [ ] Edit only the `Known Issues` sheet.
- [ ] Delete the example rows.
- [ ] Add **every genuine Part-A issue found**, including issues we may later choose to fix.
- [ ] Use the official Type/Sub Type/Severity/Reproducibility/etc. fields.
- [ ] Never include test-only issues.
- [ ] Do not invent issues.
- [ ] Put the student IDs in the workbook filename.

**Before final submission in PART D:** remove from the workbook every issue that was fixed, so the submitted file contains only unresolved issues.

## Required PART A output

Do **not** duplicate the defect list in the chat response.

Return:

1. The updated Known Issues Excel file containing the complete Part-A findings.
2. A very short summary with:
   - number of issues found;
   - number you recommend fixing before submission;
   - files changed only for comment cleanup.

Do not include learning explanations or a long code-review report.

---

# PART C — ONE-TIME WHOLE-PROJECT AUDITS

## C1. Namespace normalization

- [ ] Staff `common/` stays namespace `common`.
- [ ] Algorithm student code uses `algorithm_322889890_315113738`.
- [ ] MissionControl student code uses `mission_control_322889890_315113738`.
- [ ] UserCommon student code uses `user_common_322889890_315113738`.
- [ ] Simulator namespace follows the current skeleton/assignment rather than inventing an ID-qualified namespace.
- [ ] Declaration/definition/registration namespace spellings all match.
- [ ] No old Assignment-2 namespace remains accidentally.

## C2. Target/output naming

- [ ] `Algorithm_322889890_315113738.so`
- [ ] `MissionControl_322889890_315113738.so`
- [ ] `simulator_322889890_315113738`
- [ ] No unwanted `lib` prefix on submitted `.so` names.
- [ ] CMake/build, filesystem discovery, reports, and runtime loading all use matching names.

## C3. Project boundaries

- [ ] `common/` is staff-owned and unmodified.
- [ ] No student file was added under `common/`.
- [ ] `UserCommon/` contains only student code genuinely needed by multiple projects.
- [ ] `UserCommon/` has no standalone build project.
- [ ] Simulator, Algorithm, and MissionControl each build independently.
- [ ] Algorithm/MissionControl are not statically linked into Simulator.
- [ ] No accidental cross-project source compilation bypasses the intended separation.
- [ ] MissionControl owns/constructs its drone-control implementation; Simulator/Algorithm do not.

## C4. Registration architecture

- [ ] Concrete Algorithm has `REGISTER_MAPPING_ALGORITHM(...)` in global scope of its `.cpp`.
- [ ] Concrete MissionControl has `REGISTER_MISSION_CONTROL(...)` in global scope of its `.cpp`.
- [ ] Staff registration headers are unchanged.
- [ ] Registration implementation `.cpp` belongs to Simulator only as required.
- [ ] Algorithm/MissionControl do not depend on Simulator implementation details beyond the registration contract.

## C5. `.so` lifecycle

Review the full load → register → create → destroy instances/factories → `dlclose` lifecycle.

- [ ] Each required `.so` is loaded no more than once.
- [ ] Unloaded libraries are never loaded again.
- [ ] Every live instance is destroyed before `dlclose`.
- [ ] Every type-erased factory/callback/deleter is destroyed before `dlclose`.
- [ ] Failed/mismatched loads cannot leave stale registration state.
- [ ] Cleanup order is explicit.
- [ ] Error/exception paths obey the same guarantees.

## C6. Whole-program threading architecture

### `num_threads` semantics

- [ ] Missing `num_threads` or `num_threads=1` => main thread only.
- [ ] `num_threads>=2` => that many **additional worker threads**, plus main.
- [ ] Total thread count is never exactly 2.
- [ ] Do not create workers with nothing to execute.
- [ ] Every required simulation task runs exactly once.

### Architecture

- [ ] Work distribution is understandable.
- [ ] Shared result/storage design avoids unnecessary locking.
- [ ] Shared registrar/loader/logging/output state is synchronized.
- [ ] No unnecessary global mutex.
- [ ] No lock serializes expensive simulation work unnecessarily.
- [ ] No circular lock order or loader/registration self-deadlock.
- [ ] `dlopen` is not called while holding a mutex needed by registration callbacks.
- [ ] Main-thread waiting/join behavior cannot deadlock workers.

## C7. CLI behavior

### Comparative
Required:
- [ ] `-comparative`
- [ ] `simulation=<...>`
- [ ] `mission_control_folder=<...>`
- [ ] `algorithm=<...>`

### Competition
Required:
- [ ] `-competition`
- [ ] `simulation=<...>`
- [ ] `mission_control=<...>`
- [ ] `algorithms_folder=<...>`

### Both
- [ ] Arguments may appear in any order.
- [ ] `num_threads=<num>` is optional and validated.
- [ ] `-verbose` is optional and recognized.
- [ ] Unsupported arguments => usage + all unsupported arguments identified.
- [ ] Missing mandatory arguments => usage + missing arguments identified.
- [ ] File arguments are validated.
- [ ] Folder arguments are validated.
- [ ] A plugin folder with zero usable required files is an input error.
- [ ] Invalid CLI input exits cleanly without starting a partial simulation.

## C8. Comparative output

- [ ] Uses exactly the requested Algorithm.
- [ ] Runs all MissionControl implementations from the supplied folder.
- [ ] Runs all required simulation-composition configurations.
- [ ] Creates `comparative_results_<time>` directly under `mission_control_folder`.
- [ ] Directory name avoids collisions.
- [ ] Directory-creation failure is reported cleanly.
- [ ] Map outputs have unique traceable names.
- [ ] Required error logs are produced.
- [ ] One aggregate comparative YAML is produced.
- [ ] Each MissionControl produces its Assignment-2-style simulation-result YAML with identity in filename.
- [ ] Aggregate `results_summary` is sorted by number of agreeing managers descending.
- [ ] MissionControl implementations that cannot load/run appear in aggregate errors.

## C9. Competition output

- [ ] Uses exactly the requested MissionControl.
- [ ] Runs all Algorithm implementations from the supplied folder.
- [ ] Runs all required simulation-composition configurations.
- [ ] Creates `competition_<time>` directly under `algorithms_folder`.
- [ ] Directory name avoids collisions.
- [ ] Directory-creation failure is reported cleanly.
- [ ] Map outputs have unique traceable names.
- [ ] Required error logs are produced.
- [ ] One aggregate competitive YAML is produced.
- [ ] Each Algorithm produces its Assignment-2-style simulation-result YAML with identity in filename.
- [ ] Aggregate `results_summary` is sorted by score descending, then steps ascending.
- [ ] Algorithms that cannot load/run appear in aggregate errors.

## C10. Verbose behavior

- [ ] MissionControl creates verbose output **iff** `-verbose` was supplied.
- [ ] `-verbose` changes diagnostic/detail output only, not correctness/results.

---

# PART D — FINAL PRE-SUBMISSION CHECKLIST

## D1. Build/environment

- [ ] Build in the required Linux devcontainer/toolchain.
- [ ] All targets are warning-clean under the current project flags.
- [ ] `Algorithm/` builds independently.
- [ ] `MissionControl/` builds independently.
- [ ] `Simulator/` builds independently.
- [ ] Root build builds all three.
- [ ] Valid relative/absolute paths do not depend on hidden platform/path assumptions.

## D2. End-to-end execution

- [ ] Default happy flows finish successfully.
- [ ] Valid files/folders also work from other valid paths.
- [ ] Minor valid configuration changes do not expose hardcoded assumptions.
- [ ] Invalid/missing config/input exits gracefully with useful diagnostics.
- [ ] Normal cases finish in reasonable time.
- [ ] Comparative mode works end-to-end.
- [ ] Competition mode works end-to-end.
- [ ] Output contents, sorting, and location match Assignment 3.

## D3. Diagnostics

Debug verification only:

- [ ] Run relevant concurrent paths under ThreadSanitizer if supported.
- [ ] Run relevant ownership/lifetime paths under AddressSanitizer if supported.
- [ ] Run repeated concurrent executions and look for inconsistent results/races.
- [ ] Sanitizer flags are not required by the normal submission build.

If time is critically limited, prioritize normal build + end-to-end scenarios over optional sanitizer work.

## D4. README

- [ ] Build instructions match the real build.
- [ ] Run instructions cover both Assignment-3 modes.
- [ ] CLI options are documented correctly.
- [ ] Output folders/files are documented correctly.
- [ ] Important implementation assumptions/limitations are documented.
- [ ] Bonus behavior is documented only if actually submitted/requested.

## D5. Submission package

Final zip:

`ex3_<student1_id>_<student2_id>.zip`

Verify:

- [ ] Exactly five required folders:
  - [ ] `common`
  - [ ] `UserCommon`
  - [ ] `Algorithm`
  - [ ] `MissionControl`
  - [ ] `Simulator`
- [ ] Build file exists inside `Algorithm/`.
- [ ] Build file exists inside `MissionControl/`.
- [ ] Build file exists inside `Simulator/`.
- [ ] Root build file builds all three projects.
- [ ] Root contains `students.txt`.
- [ ] Root contains `README.md`.
- [ ] Include the completed Known Issues Excel file if submitting it.
- [ ] No student tests, test fixtures, or test-support files are included.
- [ ] No binaries are included.
- [ ] No external libraries are included.
- [ ] No development-only/scratch files are included.
- [ ] `bonus.txt` is included only if applicable.

---

# FINAL STOP CONDITIONS

Do not declare the project ready for submission if:

- [ ] `common/` differs from the staff version.
- [ ] A required interface/data type was changed.
- [ ] A project cannot build independently.
- [ ] Namespace/target/output naming is inconsistent.
- [ ] A `.so` may close while an instance/factory/callback from it is alive.
- [ ] A worker may access dead/dangling state.
- [ ] Shared mutable state lacks a synchronization/ownership explanation.
- [ ] A plausible deadlock remains.
- [ ] `num_threads` semantics differ from Assignment 3.
- [ ] CLI validation is incomplete.
- [ ] Comparative output/report behavior is incomplete.
- [ ] Competition output/report behavior is incomplete.
- [ ] Packaging does not match Assignment 3.
