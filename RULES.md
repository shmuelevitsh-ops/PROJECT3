# Assignment 3 — Project Rules

Read this file before starting every implementation stage, for every feature area.

## Namespaces
- `common/` code: namespace `common`.
- `MissionControl/common_mission_control/.../IDroneControl.h`: namespace `mission_control`.
- Simulator's own code: namespace `simulator` (not id-qualified).
- Only submitted implementation code uses id-qualified namespaces: `Algorithm_322889890_315113738`, `MissionControl_322889890_315113738`, and (if `UserCommon/` is used) `UserCommon_322889890_315113738`. The id pair applies to these namespaces and to id-qualified build target/output names (see below) — not to every namespace in the project.

## `common/` is read-only
- Nothing under `common/` (headers, `CMakeLists.txt`, or any other file) may be edited, removed, or added to — published as-is by course staff.
- `common` is an `INTERFACE` (header-only) CMake target. Never add `.cpp` files to it.

## Project boundaries and architecture
- Simulator, Algorithm, MissionControl each build independently, own `CMakeLists.txt`; none is statically linked into another. Algorithm and MissionControl are always loaded dynamically by Simulator.
- `UserCommon/` (folder name not id-qualified; its code lives in namespace `UserCommon_<ids>`) holds code shared across more than one of the three projects, per the assignment. Placing code there does not create a static link between the three deliverables — each project still only compiles what it needs from it.
- New source files go only under `Algorithm/`, `MissionControl/`, `Simulator/`, or `UserCommon/` — never inside `common/`.
- Each Mission Control provides and constructs its own drone-control implementation internally. Simulator and Algorithm must never construct a drone-control implementation directly.

## Submission structure
- Final submission: 5 folders (`common`, `UserCommon`, `Algorithm`, `MissionControl`, `Simulator`) — `UserCommon` must be present even if a given stage doesn't add code to it. 4 build files: one `CMakeLists.txt`/makefile per project (Algorithm, MissionControl, Simulator) plus one at the submission root building all three. Plus `students.txt` and `README.md` at the root.
- No binary files and no external libraries beyond the standard C++ library or ones explicitly approved on the course forum may be part of the submission.

## Build targets / naming
- Algorithm library: target/output `Algorithm_322889890_315113738` → `Algorithm_322889890_315113738.so` (`PREFIX ""`).
- MissionControl library: target/output `MissionControl_322889890_315113738` → `MissionControl_322889890_315113738.so` (`PREFIX ""`).
- Simulator executable: `simulator_322889890_315113738`.

## Threading (num_threads)
- `num_threads` missing or `1` → single thread (main thread only). `num_threads >= 2` → that many additional worker threads besides the main thread (total thread count is never exactly 2). Never open a worker thread that would have nothing to do.
- The main thread blocking on a join for all workers is fine.
- Prefer avoiding locks; where shared mutable state genuinely requires one, lock it.

## Memory / ownership discipline
- No `new`/`delete`; prefer `unique_ptr`; `shared_ptr` only for genuine shared/unknown-lifetime ownership.
- Creating an Algorithm/MissionControl instance via its factory is cheap — never cache/reuse instances.
- Default design loads each `.so` once and keeps it until final cleanup; don't design around unloading and reloading the same library mid-run. On-demand load/unload-when-idle is an optional bonus, and even there a library that was unloaded must not be loaded again.

## Dynamic loading discipline
- Safely `dlclose`ing every `.so` handle before the program ends is mandatory baseline behavior (separate from, and simpler than, the on-demand load/unload bonus above) — and must never happen while an object created from that `.so` (or anything holding type-erased code from it, e.g. a `std::function`/factory) is still alive.
- When a container owns both such type-erased state and library handles, its teardown must destroy the type-erased state first and the handles last — do not rely on member declaration order to get this right; make the order explicit.
- A failed or mismatched dynamic load must not leave any registration state referring to a library that was (or will be) unloaded as part of that same failure.

## Compiler / platform
- All targets compile under `-Wall -Wextra -Werror -pedantic` (`drone_warnings()`). Code must be warning-clean.
- POSIX/ELF-only mechanism (`dlopen`, `${CMAKE_DL_LIBS}`, `ENABLE_EXPORTS`, prefix-less `.so`). Build and verify only inside the project's Linux devcontainer/vcpkg `x64-linux` toolchain, never on native Windows.

## Scope discipline
- Touch only the files a stage's own plan explicitly names. Do not fix unrelated TODOs, warnings, or pre-existing build failures encountered along the way, and do not make an unrelated file's compilation a precondition for verifying the current stage's work, unless the active plan says otherwise.
- Do not implement optional/bonus behavior while a stage's scope is a mandatory item — note it, don't build it, unless asked.
