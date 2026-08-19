# Automatic Registration — Implementation Plan

Architecture: a single Simulator-owned `Registrar` associates each `dlopen()` call with exactly the registration(s) it triggers by diffing factory-vector sizes before/after the call, and owns library handles for the process lifetime (no per-call `dlclose`). Baseline lifecycle: load every required `.so` once, keep it mapped, `dlclose` everything only at `Registrar` teardown (this satisfies "unload before the program ends"; the assignment's early-unload/reload behavior is explicitly a bonus, not baseline). Every detail needed to implement this is spelled out per-stage below; no external reference material is required.

**Before every stage: read `ex_3_skeleton-main/RULES.md`.**

---

## Stage 1 — Registration-ready plugin libraries

**Change:** `MissionControl/src/MissionControlImpl.cpp` only.

It is unmigrated (namespace `drone_mapper`, wrong include, old multi-param constructor, no `DroneControlImpl` construction, no registration macro) and does not compile against the current `MissionControlImpl.h`.

- Namespace → `MissionControl_322889890_315113738`; include → `<MissionControl/MissionControlImpl.h>`; add `<MissionControl/DroneControlImpl.h>`, `<Common/MissionControlRegistration.h>`.
- Constructor → `explicit MissionControlImpl(common::MissionControlDependencies dependencies)`. Body: store `mission_`; bind `output_map_`; build `drone_control_ = std::make_unique<DroneControlImpl>(...)` from `dependencies` fields (verify argument order against `DroneControlImpl.h` directly, don't assume it matches `MissionControlDependencies`' field order); store `output_map_file_`, `verbose_`. No `drone_` member (none declared in the header).
- Keep the existing `runMission()` loop logic, calling through `drone_control_->step()`.
- Add `REGISTER_MISSION_CONTROL(MissionControlImpl);` at namespace scope at end of file (mirror `Algorithm/src/MappingAlgorithmImpl.cpp:754`).
- Do not implement `-verbose` output-file writing.

**Verify (no change expected):** `Algorithm/src/MappingAlgorithmImpl.cpp` already has a `common::MappingAlgorithmDependencies` constructor and `REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl);`. If it has diverged, stop and report.

**Build/verify:**
```bash
cmake --preset default
cmake --build build/default --target Algorithm_322889890_315113738
cmake --build build/default --target MissionControl_322889890_315113738
nm -D build/default/bin/Algorithm_322889890_315113738.so | c++filt | grep MappingAlgorithmRegistration
nm -D build/default/bin/MissionControl_322889890_315113738.so | c++filt | grep MissionControlRegistration
```
Expect both `.so` to build, each showing the corresponding `common::*Registration::*Registration` constructor as an **undefined (`U`)** symbol — expected until Stage 3.

**Must not change:** anything under `common/`; `Algorithm/CMakeLists.txt` / `MissionControl/CMakeLists.txt`; `Simulator/`; do not attempt to build the whole project (`simulator_322889890_315113738` stays broken for unrelated, out-of-scope reasons).

---

## Stage 2 — Simulator-side `Registrar`

**New files:** `Simulator/include/Simulator/Registrar.h`, `Simulator/src/Registrar.cpp`.

**Class `simulator::Registrar`** (Meyers' singleton via `static Registrar& instance()`, non-copyable):
```cpp
void addMappingAlgorithm(common::MappingAlgorithmFactory);
void addMissionControl(common::MissionControlFactory);
[[nodiscard]] std::vector<common::MappingAlgorithmFactory> loadMappingAlgorithm(const std::filesystem::path&);
[[nodiscard]] std::vector<common::MissionControlFactory> loadMissionControl(const std::filesystem::path&);
```
Private: nested move-only `LibraryHandle` (`dlopen` in ctor, throws `simulator::SimulationException("PLUGIN_LOAD_FAILED", dlerror())` on failure; `dlclose` in dtor iff still owning a handle); `std::mutex load_mutex_`; `std::vector<common::MappingAlgorithmFactory> mapping_algorithm_factories_`; `std::vector<common::MissionControlFactory> mission_control_factories_`; `std::vector<LibraryHandle> libraries_`.

**Required decisions:**
- `addX` **appends** (push_back), never overwrites, and **must not lock `load_mutex_`**. `addX` is only ever called synchronously from inside `dlopen()`, on the same thread that already holds `load_mutex_` for the whole `loadX()` call (see below). `load_mutex_` is a plain `std::mutex` (non-recursive); if `addX` also locked it, that re-lock by the same thread is a self-deadlock, not a race. Do not "fix" this with `std::recursive_mutex` — instead preserve the invariant that `dlopen` on an Algorithm/MissionControl `.so` is only ever called from `Registrar::loadX()`, so every `addX` call is guaranteed to already be inside that lock.
- `loadMappingAlgorithm(path)`: lock `load_mutex_` for the whole call → record `mapping_before = mapping_algorithm_factories_.size()` **and** `mission_before = mission_control_factories_.size()` → construct `LibraryHandle` (dlopen; on dlopen failure the exception propagates, nothing to clean up) → if `mapping_algorithm_factories_.size() == mapping_before` (this `.so` did not register a mapping algorithm): `mission_control_factories_.resize(mission_before)` (discard any stray cross-type registration) **then** throw `SimulationException("PLUGIN_NOT_REGISTERED", ...)` — the local `LibraryHandle` self-closes via RAII during unwind, and by construction no factory anywhere still references it. Else: `mission_control_factories_.resize(mission_before)` defensively (a `.so` must register exactly one type), `libraries_.push_back(std::move(library))` (kept until process exit — do not `dlclose` here), return the slice `[mapping_before, end())`. `loadMissionControl` is symmetric.
- `~Registrar()`: explicit body `mapping_algorithm_factories_.clear(); mission_control_factories_.clear(); libraries_.clear();` — factories destroyed before handles, stated explicitly (do not rely on member declaration order).

**CMake:** add `src/Registrar.cpp` directly to `Simulator/CMakeLists.txt`'s `add_executable(simulator_322889890_315113738 ...)` sources (per RULES — that target still won't build as a whole; expected).

**Verify:**
- No-dlopen check: `addMappingAlgorithm` on a local lambda directly (not via `loadX`), confirm it appends without touching `load_mutex_` itself (e.g. call it while the test already holds `load_mutex_` via a friend/test hook, or simply reason from the code that it contains no lock) — not required to be kept as a permanent test.
- `LibraryHandle`/`loadX` failure path, self-contained (no Algorithm/MissionControl `.so` needed yet — full registration integration is Stage 3's job): call `loadMappingAlgorithm("<any nonexistent path>.so")` and assert it throws `simulator::SimulationException` with `.code() == "PLUGIN_LOAD_FAILED"`; call it a second time with another bad path and confirm it also throws promptly (proves `load_mutex_` was released after the first failure, not left locked). This exercises `LibraryHandle`'s constructor failure branch and confirms no handle is leaked/leftover when `dlopen` itself fails.

**Must not change:** `common/`; Algorithm/MissionControl sources; do not add per-load-call `dlclose` (baseline unloads only at process end).

---

## Stage 3 — Registration-constructor wiring + first real load

**New files:** `Simulator/src/MappingAlgorithmRegistration.cpp`, `Simulator/src/MissionControlRegistration.cpp`.

```cpp
// Simulator/src/MappingAlgorithmRegistration.cpp
namespace common {
MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::Registrar::instance().addMappingAlgorithm(std::move(factory));
}
}
```
Symmetric for `MissionControlRegistration`/`addMissionControl`.

**Required decision:** these two `.cpp` files must be **direct sources** of every executable that performs loading (`simulator_322889890_315113738` now, the Stage-4 test executable later) — never factored into a separate static/OBJECT library. Nothing in the executable's own code calls these constructors directly (only a dynamically-loaded `.so`'s global constructor does), so a linker pulling objects from an archive by referenced symbol would silently drop them, breaking `dlopen` resolution at runtime despite a clean build.

**CMake:** add both files to `Simulator/CMakeLists.txt`'s `add_executable(simulator_322889890_315113738 ...)` sources. Confirm (don't re-add) `${CMAKE_DL_LIBS}` and `ENABLE_EXPORTS ON` are already present on that target.

**Verify:** a minimal ad hoc program (can become part of Stage 4's test) that calls `simulator::Registrar::instance().loadMappingAlgorithm("<build>/bin/Algorithm_322889890_315113738.so")` and `loadMissionControl("<build>/bin/MissionControl_322889890_315113738.so")` against Stage 1's built libraries. Assert only: the call does not throw (no unresolved-symbol/`dlopen` failure, no `PLUGIN_NOT_REGISTERED`), and the returned vector has size exactly 1 with a non-empty (`operator bool() == true`) factory of the expected type. Do **not** call the factory or construct a `MappingAlgorithmDependencies`/`MissionControlDependencies` here — that needs real mission/lidar/drone configs and mocks, which is Stage 4's integration work.

**Failure modes:** undefined-symbol at `dlopen` time → missing `ENABLE_EXPORTS` or the archive-drop issue above (check `dlerror()` text). Loading the wrong plugin type must throw `PLUGIN_NOT_REGISTERED` cleanly (exercised fully in Stage 4).

**Must not change:** `drone_mapper_simulation_main.cpp`, `SimulationRunFactoryImpl.cpp`; no folder-scanning/orchestration logic (future work).

---

## Stage 4 — End-to-end verification (GTest)

**New file:** `Simulator/tests/registration/registration_end_to_end_test.cpp`.

**CMake:** add `find_package(GTest CONFIG REQUIRED)` to `Simulator/CMakeLists.txt` (not yet declared there). New `add_executable(simulator_registration_test ...)` with sources: the new test file, `Registrar.cpp`, `MappingAlgorithmRegistration.cpp`, `MissionControlRegistration.cpp`, plus existing `ConfigLoader.cpp`, `Map3DImpl.cpp`, `MockGPS.cpp`, `MockLidar.cpp`, `MockMovement.cpp` (do not include `SimulationManager.cpp`, `SimulationRunFactoryImpl.cpp`, `SimulationRunImpl.cpp`, `SimulationOutputWriter.cpp`, `MapsComparison.cpp`, or `drone_mapper_simulation_main.cpp`). Link `common::common yaml-cpp::yaml-cpp TinyNPY::TinyNPY Threads::Threads ${CMAKE_DL_LIBS} GTest::gtest GTest::gtest_main`; `ENABLE_EXPORTS ON`; `target_compile_definitions` with `ALGORITHM_PLUGIN_PATH="$<TARGET_FILE:Algorithm_322889890_315113738>"`, `MISSION_CONTROL_PLUGIN_PATH="$<TARGET_FILE:MissionControl_322889890_315113738>"`, `TEST_INPUTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../inputs"`; `add_dependencies` on both `.so` targets. Run the binary directly (matches `FILES PROJECT 2/CMakeLists.txt` convention). Test-*case* discovery via `enable_testing()`/CTest/`gtest_discover_tests` is intentionally not used — `GTest::gtest_main` supplies the binary's own `main()`.

**Why the test structure below is not just "one `loadX()` call per `TEST_F`":** `dlopen` on an already-resident `.so` path does not rerun that library's global constructors — a second `loadX()` call on a path already loaded in this process sees no new registration and throws `PLUGIN_NOT_REGISTERED`, indistinguishable from a genuine type mismatch. Since `Registrar` is a process-wide singleton and every `TEST_F` in this binary shares one process, each real plugin path must be `dlopen`'d for the first time exactly once, in a controlled order, not once per independent `TEST_F`.

**Fixture `RegistrationEndToEnd` — required structure:**
- `SetUpTestSuite()` (runs once, before any test; this exact order is load-bearing):
  1. Build a fresh empty temp directory (`std::filesystem::temp_directory_path() / "simulator_registration_test"`, `remove_all` then `create_directories`); store it in a static member.
  2. For `ALGORITHM_PLUGIN_PATH`: first call `loadMissionControl(ALGORITHM_PLUGIN_PATH)` inside a `try`/`catch` — this is the path's first-ever `dlopen` in the process, so its constructor genuinely runs and registers as a mapping algorithm, not a mission control, so this must throw; store the caught `SimulationException::code()` in a static string (don't `ASSERT_*` here — assertions belong in a `TEST_F`, not `SetUpTestSuite`). Then call the real `loadMappingAlgorithm(ALGORITHM_PLUGIN_PATH)` and store the returned factory in a static member. This second call is again a genuine first load, because step 2's failed probe fully `dlclose`d the library (Stage 2's failure path never reaches `libraries_.push_back`).
  3. Same two-step sequence, roles swapped, for `MISSION_CONTROL_PLUGIN_PATH` (`loadMappingAlgorithm` probe expected to throw, then the real `loadMissionControl`).
- `TearDownTestSuite()`: **first** reset the two stored factory statics (e.g. `mapping_factory_.reset(); mission_factory_.reset();`), **then** `std::filesystem::remove_all(temp_dir)`. This reset is required, not cosmetic: these are `static` fixture data members, so they have static storage duration and are destroyed at final program teardown. `Registrar`'s own singleton is a function-local static, first constructed only when `SetUpTestSuite()` makes its first `Registrar::instance()` call — i.e. strictly after these fixture statics already exist — so by C++'s reverse-of-construction-order rule, `Registrar::~Registrar()` (which `dlclose`s every library) runs *before* the fixture's statics are destroyed, not after. Each stored factory is a copy of an entry `Registrar`'s own internal vector also still holds, and both copies point at code living in the `.so`; left unreset, the fixture's copies would be destroyed after their library was already `dlclose`d — a use-after-unload. Resetting them here destroys those specific copies while the library is still mapped; the now-empty statics are a no-op to destroy later.
- No `TEST_F` calls `loadMappingAlgorithm`/`loadMissionControl` itself — each uses the factories/codes/temp dir `SetUpTestSuite` already produced. Created instances (the `unique_ptr<IMappingAlgorithm>`/`unique_ptr<IMissionControl>` returned by a factory) must stay local to the `TEST_F` body that creates them, never stored as fixture statics — the same use-after-unload risk applies to them.

**Test cases:**
1. `RejectsMismatchedPluginType` — asserts both codes captured in `SetUpTestSuite` equal `"PLUGIN_NOT_REGISTERED"`.
2. `CreatesMappingAlgorithmInstance` — using the stored Algorithm factory, build a real `MappingAlgorithmDependencies` from `TEST_INPUTS_DIR` (`drone/drone_small.yaml`, `lidar/lidar_short.yaml`, `mission/small_mission_room.yaml`, `map/scenario_small.npy`) via `ConfigLoader`/`Map3DImpl`; create an instance; one `nextStep()` call; assert no throw and a non-null instance.
3. `CreatesMissionControlInstance` — using the stored MissionControl factory plus a freshly-created Algorithm instance (call the stored Algorithm factory again; don't reuse test 2's instance). Parse the same mission config but override `max_steps` to a small constant (e.g. `2`) before building `MissionControlDependencies`, so `runMission()` exercises only the create → step → save plumbing, not real mapping-algorithm completion. Write `output_map_file` under the suite's temp dir. Call `runMission()` once; assert no throw and `std::filesystem::exists(output_map_file)` afterward.

No separate "unloads at process end" case: a crash during `Registrar`'s static teardown happens after every `TEST_F` has already reported its result, so GTest cannot attribute it to any specific case — it's a property of the whole run, checked below instead.

**Verify:**
```bash
cmake --build build/default --target simulator_registration_test
build/default/bin/simulator_registration_test; echo "exit=$?"
```
Expect all `TEST_F` cases to pass **and** `exit=0` — a nonzero/signal exit after GTest's summary already printed all-green indicates a crash during `Registrar`'s static destruction (factories-before-handles ordering, Stage 2).

**Failure modes:** `create()`/`runMission()` throwing or returning null is not necessarily a Stage 1 constructor mismatch — also check a dangling reference in the test's own hand-built `Dependencies` (its fields are references; the `Map3DImpl`/`Mock*` locals they point at must outlive every use of the created instance — check declaration/destruction order in the test), a genuine `MappingAlgorithmImpl`/`MissionControlImpl` logic bug unrelated to registration (out of this plan's scope — report, don't fix algorithm behavior here), or malformed/inconsistent values in the `TEST_INPUTS_DIR` config files. `SetUpTestSuite` step 2/3 not throwing at all means Stage 2's type-diff check itself is broken, not that a file path is wrong.

**Must not change:** `drone_mapper_simulation_main.cpp`, `SimulationRunFactoryImpl.cpp`; no `enable_testing()`/CTest wiring; no weakening of `-Werror`.
