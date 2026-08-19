# Assignment 3 — RULES

This file is intentionally split into **four different workflows**.  
Do not mix them.

---

# 0. How to use this file

## Workflow A — Per-file RULES review
Run this **every time we start reading a new student-owned source/header file**.

Goal:
- detect actual rule/style/design violations in that file;
- explain them;
- propose the smallest fix if needed.

Do **not** modify code unless explicitly asked.

Use **PART A**.

---

## Workflow B — Per-file "things I need to know exist" report
Run this **for the same file, immediately after Workflow A**.

Goal:
- tell me which C++ features/concepts from the lectures appear in the file;
- explain where they appear and why they are used;
- help me learn the code.

This workflow is **report-only**:
- do not change code;
- do not recommend adding a feature merely because it was taught;
- absence of a template/design-pattern/atomic/etc. is not a violation.

Use **PART B**.

---

## Workflow C — One-time whole-project audits / cleanup tasks
These are separate, dedicated passes over the repository.

Examples:
- fix/verify namespaces everywhere;
- verify target/output names everywhere;
- verify registration placement;
- verify project boundaries / `common` / `UserCommon`;
- audit threading architecture across files;
- audit `.so` ownership/lifetime across files.

These should **not** be repeated as a full-project operation every time we read one file.

Use **PART C**.

---

## Workflow D — Final pre-submission audit
Run only when implementation and code-reading are effectively complete.

Goal:
- build/run checks;
- CLI/output validation;
- sanitizer/concurrency checks;
- README and packaging;
- final repository cleanliness.

Use **PART D**.

---

# Source priority

If anything conflicts:

1. Current Assignment 3 instructions.
2. Current course skeleton and staff-provided `common/` headers.
3. Assignment 2 rules that Assignment 3 did not redefine.
4. Course review/grading guidance.
5. Lecture notes / this learning inventory.

The lecture-feature inventory in PART B is **not** a specification.

This file intentionally contains no requirements about submitting a student test suite.

---

# PART A — PER-FILE RULES REVIEW

Run this entire part for **every student-owned `.h`, `.hpp`, and `.cpp` file**.

For rules that are irrelevant to the current file, mark them `N/A`; do not invent work.

Recommended agent output:

```text
FILE: path/to/file.cpp

A. RULE VIOLATIONS
- file:line — rule — why it is a problem — minimal suggested fix

If none:
- No RULES violations found in this file.

B. Then run PART B separately.
```

---

## A1. File boundaries and dependencies

- [ ] The file belongs in the correct project: `Simulator/`, `Algorithm/`, `MissionControl/`, or `UserCommon/`.
- [ ] No student implementation exists under `common/`.
- [ ] The file does not modify/redeclare staff-provided interfaces or data types.
- [ ] Code shared by more than one student-owned project is not duplicated when it belongs in `UserCommon/`.
- [ ] Every `#include` is actually required. No unnecessary include remains. `(e17)`
- [ ] The file does not introduce unnecessary coupling/include cycles.
- [ ] No unapproved or manually installed external dependency is required. `(e11)`
- [ ] No broad `using namespace ...;` appears in implementation code.
- [ ] Constants use typed `constexpr` / `const`; no `#define` is used merely for a constant.

---

## A2. Types and parameter passing

- [ ] Physical/domain values use the project's strong types where such a type exists. `(e16)`
- [ ] `mp-units` quantities remain wrapped during actual calculations. `(e03)`
- [ ] Quantity values are converted to raw numbers only at genuine boundaries such as serialization, indexing, or APIs requiring raw numeric values.
- [ ] Non-modified, non-trivial parameters are passed by `const&` rather than unnecessary copy or mutable reference. `(e06)`
- [ ] Small cheap scalar/value types are passed by value where appropriate.
- [ ] A pointer is used only when nullability is meaningful.
- [ ] A pointer that may be null is checked before dereference. `(e13)`
- [ ] A reference is not used when the referenced lifetime cannot be guaranteed.
- [ ] No C-style casts are used.
- [ ] `static_cast` / `dynamic_cast` are used only when the conversion is intentional and justified.
- [ ] void* is used only at a justified low-level/API boundary and not as ordinary domain-code typing


---

## A3. Const-correctness and encapsulation

- [ ] Methods that do not mutate observable state are `const` where the interface permits it. `(e07)`
- [ ] Data members are private unless public access is genuinely required. `(e22)`
- [ ] There is no unnecessary public API. `(e08)`
- [ ] Implementation-only helpers/types are file-local, preferably in an anonymous namespace.
- [ ] A header contains one cohesive component rather than unrelated classes. `(e01)`
- [ ] The design is not fragmented into needless tiny headers/classes. `(e02)`

---

## A4. Functions and implementation quality

- [ ] Each function has one clear responsibility.
- [ ] A long function is reviewed critically; roughly `>40` lines requires a good reason or extraction into helpers. `(e09)`
- [ ] Logic repeated in multiple places is extracted when appropriate. `(e10)`
- [ ] Expensive work is not recomputed needlessly. `(e21)`
- [ ] The STL container/algorithm used is appropriate for the operation and expected complexity. `(e04)`
- [ ] No magic number substitutes for configuration, bounds, IDs, sizes, timeouts, or assignment-defined values. `(e23)`
- [ ] Comments/documentation are in English.
- [ ] Comments explain non-obvious intent/reasoning rather than restating syntax.
- [ ] No stale dead code or commented-out implementation remains.
- [ ] If the code has a recurring design problem that a standard design pattern would clearly simplify,
      consider whether that pattern would reduce coupling or complexity.
      Do not recommend a pattern merely to demonstrate one.

---

## A5. Ownership and memory

- [ ] There is no `new`.
- [ ] There is no `delete`.
- [ ] There is no `malloc`, `calloc`, `realloc`, or `free`.
- [ ] `std::unique_ptr` is the default owning smart pointer.
- [ ] `std::shared_ptr` exists only for genuine shared ownership where lifetime is not owned by one clear object.
- [ ] Raw pointers are non-owning only.
- [ ] Resource ownership is obvious from the code.
- [ ] Every resource uses RAII where possible.
- [ ] No error path leaks ownership or bypasses required cleanup.
- [ ] No unnecessary global or Singleton mutable state; if such state exists, its ownership, lifetime, and thread-safety are explicit and justified.
---

## A6. Constructors, copying, moving, and Rule of Zero/Three/Five

- [ ] Prefer the **Rule of Zero** when the class owns no resource requiring custom lifecycle code.
- [ ] If a class defines a custom destructor/copy/move operation, review the complete Rule of Three/Five implications.
- [ ] Copying is explicitly deleted when copying the object would duplicate ownership incorrectly.
- [ ] Move operations leave the source object valid/destructible.
- [ ] Move operations are `noexcept` when they can truthfully be `noexcept`.
- [ ] Constructors initialize members in the initializer list where appropriate.
- [ ] References and non-default-constructible members are initialized correctly.
- [ ] Initialization logic respects actual **member declaration order**, not the textual order of the initializer list.
- [ ] Every `std::move` has a reason.
- [ ] Code does not move from an object and then incorrectly assume its old value remains.
- [ ] `const` objects are not `std::move`d expecting a real move.
- [ ] `T&&` / `std::forward` are used only with the correct rvalue/forwarding semantics.

---

## A7. RAII and cleanup

- [ ] Memory ownership is RAII-managed.
- [ ] Lock ownership is RAII-managed.
- [ ] File/stream redirection is restored automatically on all paths.
- [ ] Dynamic-library handles have clear ownership and cleanup.
- [ ] Cleanup still occurs when an exception/error causes an early return.
- [ ] Normal error handling does not use `std::exit` in a way that bypasses destructors/cleanup.

---

## A8. Error handling

- [ ] Errors are handled at the layer that detects them.
- [ ] Required diagnostics are emitted immediately rather than carried through unrelated layers only to be logged later. `(e05)`
- [ ] A recoverable per-run error does not unnecessarily terminate unrelated simulation work.
- [ ] Assignment-2 inherited behavior is preserved where still applicable: an isolatable failed run receives error score `-1` and other runnable cases may continue.
- [ ] Invalid input/configuration does not cause an uncontrolled crash.
- [ ] No `catch` silently swallows an error that the program is required to report.
- [ ] Failure does not leave half-created registration/output/shared state behind.

---

## A9. Thread-safety rules — apply when this file participates in concurrent execution

These are still RULES, but they may be `N/A` for a file unrelated to threading/shared state.

### Thread lifetime

- [ ] Prefer `std::jthread` for owned workers unless there is a concrete reason not to.
- [ ] Any `std::thread` has guaranteed `join()`/lifetime handling on every path.
- [ ] `detach()` is used only with a strong documented lifetime reason.
- [ ] No worker can outlive referenced data, callbacks, factories, or owners it accesses.
- [ ] Exceptions in worker work are caught/translated into the program's normal result/error mechanism rather than escaping the thread function.

### Thread arguments

- [ ] Code accounts for the fact that thread arguments/captures may be copied or moved.
- [ ] `std::ref` / `std::cref` is used only when sharing the original object is intentional and lifetime-safe.
- [ ] A function taking `const&` is not accidentally receiving a copied thread argument when the programmer intended shared access.
- [ ] Lambda captures used by workers have safe lifetimes.

### Shared state

- [ ] Every object read/written by multiple threads has an explicit synchronization or ownership strategy.
- [ ] Shared mutable state is avoided when per-task/per-slot state can remove the need for synchronization.
- [ ] Result storage is preallocated/per-task when that safely avoids a lock.
- [ ] Shared registrar/loader state is synchronized.
- [ ] Shared logging/output state is synchronized where necessary.
- [ ] `volatile` is not used as a synchronization mechanism.
- [ ] Atomics are used only for state that is genuinely representable atomically; compound invariants are not incorrectly implemented as several unrelated atomics.
- [ ] `thread_local` is used only for truly per-thread state.

### Locks

- [ ] A lock exists only where shared mutable state genuinely requires it.
- [ ] Use RAII locks such as `std::lock_guard`, `std::scoped_lock`, or `std::unique_lock` when appropriate.
- [ ] Avoid manual `lock()` / `unlock()` where an RAII lock can be used.
- [ ] Critical sections are as small as practical.
- [ ] Expensive work / I/O / plugin execution / callbacks are not performed under a lock unless necessary.
- [ ] A mutex is preferably owned by the relevant class/object rather than being global.
- [ ] Multiple-mutex acquisition has a deadlock-safe strategy/order.
- [ ] No path acquires the same non-recursive mutex twice on the same thread.
- [ ] Destruction/cleanup does not wait for a worker while holding a lock that the worker needs.

### Condition variables

- [ ] `condition_variable::wait` uses the appropriate mutex/`unique_lock`.
- [ ] Waits use a predicate and correctly tolerate spurious wakeups.
- [ ] Notification is performed in a sensible lock/unlock order.

---

## A10. Dynamic-loading rules — apply when the file participates in `.so` loading/registration

- [ ] Registration uses the staff-provided mechanism rather than a parallel custom API.
- [ ] Successfully opened `.so` handles are eventually closed.
- [ ] `dlclose` never occurs while an object created by that library is alive.
- [ ] `dlclose` never occurs while a factory / `std::function` / callback / deleter containing code from that library is alive.
- [ ] Type-erased factory/registration state is destroyed before the library handle.
- [ ] Correct teardown order is explicit rather than accidental.
- [ ] A failed load leaves no stale registration/factory referring to a library that is being discarded.
- [ ] A `.so` handle owner is not accidentally copyable.
- [ ] A library is not unloaded and later loaded again.
- [ ] Algorithm/MissionControl **instances** may be recreated cheaply from their factories; they should not be cached merely to avoid construction.

---

# PART B — PER-FILE LEARNING / FEATURE INVENTORY

Run this after PART A for every file we read.

This part **never creates a code change by itself**.

Recommended agent output:

```text
FILE: path/to/file.cpp

FEATURES TO KNOW IN THIS FILE

- Feature: std::move
  Location: lines ...
  What it means here:
  Why this code uses it:
  What I should be able to explain:

- Feature: lambda
  ...

NOT PRESENT IN THIS FILE
- templates
- condition_variable
- ...
```

The report should distinguish:
- **present and important**;
- **present but trivial**;
- **not present**.

Do not suggest adding a missing feature.

---

## B1. Design and abstraction features

Report any occurrence of:

- [ ] Design pattern.
- [ ] Factory.
- [ ] Automatic registration pattern.
- [ ] RAII wrapper/guard.
- [ ] Singleton, if any.
- [ ] Other recognizable design pattern.

For each:
- where;
- what problem it solves;
- why it is preferable to a simpler alternative **in this code**.

---

## B2. Templates and compile-time programming

Report any occurrence of:

- [ ] Function template.
- [ ] Class template.
- [ ] Variadic template.
- [ ] Function parameter declared with `auto` (abbreviated function template).
- [ ] `requires`.
- [ ] Concept.
- [ ] `if constexpr`.
- [ ] `std::forward`.

Important:
- A template does **not** automatically need a concept.
- Absence of templates/concepts is not a problem.
- Do not convert ordinary code to templates just to demonstrate templates.

---

## B3. Pointers, references, ownership, values

Report examples of:

- [ ] Raw pointer.
- [ ] Reference.
- [ ] `const&`.
- [ ] `std::unique_ptr`.
- [ ] `std::shared_ptr`.
- [ ] `std::move`.
- [ ] lvalue/rvalue use.
- [ ] `T&&`.
- [ ] Copy constructor / copy assignment.
- [ ] Move constructor / move assignment.
- [ ] Custom destructor.

For each relevant occurrence explain:
- owning or non-owning?
- who controls lifetime?
- why pointer vs reference vs value?
- why copy vs move?

---

## B4. Functions / language syntax worth recognizing

Report examples of:

- [ ] Lambda.
- [ ] Static function/member.
- [ ] Const member function.
- [ ] Iterator/range.
- [ ] `static_cast`.
- [ ] `dynamic_cast`.
- [ ] `void*`.
- [ ] important standard-library algorithms/containers/utilities.

For `void*`:
- distinguish legitimate POSIX/dynamic-loading boundary use from ordinary domain-code use.

---

## B5. Threading concepts worth recognizing

Report any occurrence of:

- [ ] `std::jthread`.
- [ ] `std::thread`.
- [ ] `join`.
- [ ] `detach`.
- [ ] `std::ref` / `std::cref`.
- [ ] Mutex.
- [ ] `std::lock_guard`.
- [ ] `std::scoped_lock`.
- [ ] `std::unique_lock`.
- [ ] `condition_variable`.
- [ ] Atomic variable.
- [ ] `thread_local`.
- [ ] Thread pool / worker-pool pattern.
- [ ] Reader/writer locking.
- [ ] RCU-style idea, if any.

For every threading feature found, explain:
- what threads can access it;
- what is shared;
- why synchronization is or is not required;
- what race/deadlock issue it is preventing.

---

# PART C — ONE-TIME WHOLE-PROJECT AUDITS

These are **dedicated repository-wide passes**.  
Do not mix them into the ordinary file-reading loop.

---

## C1. Namespace normalization pass

Run once across all code.

Verify/fix:

- [ ] Staff `common/` stays namespace `common`.
- [ ] Algorithm student code uses `algorithm_322889890_315113738`.
- [ ] MissionControl student code uses `mission_control_322889890_315113738`.
- [ ] UserCommon student code uses `user_common_322889890_315113738`.
- [ ] Simulator namespace follows the current skeleton/assignment rather than inventing an ID-qualified namespace.
- [ ] Declaration/definition/registration namespace spellings all match.
- [ ] No old Assignment-2 namespace remains accidentally.

This is a **repository migration task**, not a per-file learning observation.

---

## C2. Target/output naming pass

Run once across build files and registration/loading code.

- [ ] `Algorithm_322889890_315113738.so`
- [ ] `MissionControl_322889890_315113738.so`
- [ ] `simulator_322889890_315113738`
- [ ] No unwanted `lib` prefix on submitted `.so` names.
- [ ] Names used by CMake/make, filesystem discovery, reports, and runtime loading agree.

---

## C3. Project-boundary pass

Run once over repository structure/build files.

- [ ] `common/` is byte-for-byte staff-owned / unmodified.
- [ ] No student file was added under `common/`.
- [ ] `UserCommon/` contains only student code genuinely needed by multiple projects.
- [ ] `UserCommon/` has no standalone makefile/CMake project.
- [ ] Simulator, Algorithm, MissionControl each build independently.
- [ ] Algorithm/MissionControl are not statically linked into Simulator.
- [ ] No accidental cross-project source compilation bypasses the intended separation.
- [ ] MissionControl owns/constructs its drone-control implementation; Simulator/Algorithm do not.

---

## C4. Registration architecture pass

- [ ] Concrete Algorithm has `REGISTER_MAPPING_ALGORITHM(...)` in global scope of its `.cpp`.
- [ ] Concrete MissionControl has `REGISTER_MISSION_CONTROL(...)` in global scope of its `.cpp`.
- [ ] Staff registration headers are unchanged.
- [ ] Registration implementation `.cpp` belongs to Simulator only as required by the assignment.
- [ ] Algorithm/MissionControl do not depend on Simulator implementation details beyond the registration contract.

---

## C5. `.so` lifecycle pass

Review the complete load → register → create instances → destroy instances/factories → `dlclose` lifecycle across files.

- [ ] Each required `.so` is loaded no more than once.
- [ ] Unloaded libraries are never loaded again.
- [ ] Every live instance is destroyed before the relevant `dlclose`.
- [ ] Every type-erased factory/callback/deleter is destroyed before `dlclose`.
- [ ] Failed/mismatched loads cannot leave stale registration state.
- [ ] Cleanup order is explicit and understandable.
- [ ] Exception/error paths obey the same lifecycle guarantees.

---

## C6. Whole-program threading architecture pass

Run after the threaded architecture exists.

### Assignment-3 `num_threads` semantics

- [ ] Missing `num_threads` or `num_threads=1` => main thread only.
- [ ] `num_threads>=2` => that many **additional worker threads**, plus main.
- [ ] Total thread count is never exactly 2.
- [ ] Do not create workers that have nothing to execute.
- [ ] Every required simulation task runs exactly once.

### Architecture audit

- [ ] Work distribution is understandable and deterministic in responsibility.
- [ ] Shared result/storage design avoids unnecessary locking.
- [ ] All shared registrar/loader/logging/output state has a synchronization strategy.
- [ ] No global mutex exists without a strong reason.
- [ ] No unnecessary lock serializes the expensive simulation work.
- [ ] There is no circular lock order.
- [ ] There is no loader/registration self-deadlock.
- [ ] In particular, check whether `dlopen` is called while holding a mutex that registration callbacks may need.
- [ ] Main-thread waiting/join behavior cannot deadlock workers.

---

## C7. CLI behavior pass

### Comparative mode

Required:
- [ ] `-comparative`
- [ ] `simulation=<...>`
- [ ] `mission_control_folder=<...>`
- [ ] `algorithm=<...>`

### Competition mode

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
- [ ] File arguments are validated for existence/openability.
- [ ] Folder arguments are validated for existence/traversability.
- [ ] A plugin folder containing zero usable files of the required kind is an input error.
- [ ] Invalid CLI input exits cleanly without starting a partial simulation.

---

## C8. Comparative output pass

- [ ] Uses exactly the requested Algorithm.
- [ ] Runs all MissionControl implementations from the supplied folder.
- [ ] Runs them across all simulation-composition configurations required by the assignment.
- [ ] Creates `comparative_results_<time>` directly under `mission_control_folder`.
- [ ] Directory name avoids collisions.
- [ ] Failure to create the directory is reported cleanly.
- [ ] All map outputs have unique names traceable to their run.
- [ ] Required error logs are produced.
- [ ] One aggregate comparative report YAML is produced.
- [ ] Each MissionControl also produces its Assignment-2-style simulation-result YAML with its identity in the filename.
- [ ] Aggregate `results_summary` is sorted by number of agreeing managers descending.
- [ ] MissionControl implementations that cannot be loaded/run appear in the aggregate errors list.

---

## C9. Competition output pass

- [ ] Uses exactly the requested MissionControl.
- [ ] Runs all Algorithm implementations from the supplied folder.
- [ ] Runs them across all simulation-composition configurations required by the assignment.
- [ ] Creates `competition_<time>` directly under `algorithms_folder`.
- [ ] Directory name avoids collisions.
- [ ] Failure to create the directory is reported cleanly.
- [ ] All map outputs have unique names traceable to their run.
- [ ] Required error logs are produced.
- [ ] One aggregate competitive report YAML is produced.
- [ ] Each Algorithm also produces its Assignment-2-style simulation-result YAML with its identity in the filename.
- [ ] Aggregate `results_summary` is sorted by score descending, then steps ascending.
- [ ] Algorithms that cannot be loaded/run appear in the aggregate errors list.

---

## C10. Verbose behavior pass

- [ ] MissionControl creates verbose output **iff** `-verbose` was supplied.
- [ ] `-verbose` changes diagnostic/detail output only, not simulation correctness/results.

---

# PART D — FINAL PRE-SUBMISSION CHECKLIST

Run only near submission.

---

## D1. Build/environment

- [ ] Build in the required Linux devcontainer/toolchain.
- [ ] Do not validate final behavior using native Windows as the authoritative environment.
- [ ] All targets are warning-clean under the current project flags (`-Wall -Wextra -Werror -pedantic` or skeleton equivalent).
- [ ] `Algorithm/` builds independently.
- [ ] `MissionControl/` builds independently.
- [ ] `Simulator/` builds independently.
- [ ] Root build builds all three.
- [ ] No hidden path/platform assumption breaks valid relative/absolute inputs.

---

## D2. End-to-end program execution

- [ ] Default provided happy flows finish successfully.
- [ ] Valid scenarios also work when files/folders are supplied from another valid path.
- [ ] Minor valid configuration changes do not expose hardcoded assumptions.
- [ ] Invalid/missing config or input exits gracefully with useful diagnostics.
- [ ] All normal cases finish in reasonable time.
- [ ] Multithreading provides concurrency without an avoidable global serial bottleneck.
- [ ] Comparative mode works end-to-end.
- [ ] Competition mode works end-to-end.
- [ ] Output report contents/sorting/location match Assignment 3.

---

## D3. Sanitizer / race diagnostics

Debug verification only; do not accidentally make these flags mandatory in the submission build.

- [ ] Run relevant concurrent paths under ThreadSanitizer (`-fsanitize=thread`) if supported by the project toolchain.
- [ ] Run relevant ownership/lifetime paths under AddressSanitizer (`-fsanitize=address`) if supported.
- [ ] Run repeated concurrent executions and check for unstable/inconsistent results that suggest a race.
- [ ] No sanitizer-only configuration remains required by the normal build.

---

## D4. README

- [ ] README build instructions match the real build.
- [ ] README run instructions match both Assignment-3 modes.
- [ ] CLI options are documented correctly.
- [ ] Output folders/files are documented correctly.
- [ ] Any implementation-specific assumptions/limitations worth telling the grader are documented.
- [ ] Bonus behavior is documented only if actually submitted/requested.

---

## D5. Submission package

Final zip must be named:

`ex3_<student1_id>_<student2_id>.zip`

Verify:

- [ ] Exactly five required folders are included:
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
- [ ] No binary files are included.
- [ ] No external libraries are included.
- [ ] No development-only/scratch folders/files are included.
- [ ] `bonus.txt` is included only if applicable.

---

# FINAL STOP CONDITIONS

Do not declare the project ready for submission if any of the following remains true:

- [ ] `common/` differs from the staff version.
- [ ] A required interface/data type was changed.
- [ ] A project cannot build independently.
- [ ] Namespace/target/output naming is inconsistent.
- [ ] A `.so` may be closed while an instance/factory/callback from it is alive.
- [ ] A worker may access dead/dangling state.
- [ ] Shared mutable state lacks a synchronization/ownership explanation.
- [ ] A plausible deadlock path remains.
- [ ] `num_threads` semantics differ from Assignment 3.
- [ ] CLI validation is incomplete.
- [ ] Comparative output/report behavior is incomplete.
- [ ] Competition output/report behavior is incomplete.
- [ ] Packaging does not match Assignment 3.
- [ ] There is code in the submission that we cannot confidently explain in the post-submission review.
