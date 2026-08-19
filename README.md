# Assignment 3 - Drone Mapper

This is the core skeleton for assignment 3. You should update this README file.

Use the lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator` in your implementation.

## Provided file tree

```text
.
|-- .devcontainer/...
|-- Algorithm/
|   |-- CMakeLists.txt
|   |-- include/Algorithm/
|   `-- src/
|-- MissionControl/
|   |-- CMakeLists.txt
|   |-- common_mission_control/include/MissionControl/IDroneControl.h
|   |-- include/MissionControl/
|   `-- src/
|-- Simulator/
|   |-- CMakeLists.txt
|   |-- common_simulator/include/Simulator/
|   |   |-- ISimulation.h
|   |   |-- ISimulationRun.h
|   |   |-- ISimulationRunFactory.h
|   |   `-- SimulationTypes.h
|   |-- include/Simulator/
|   `-- src/
|-- common/
|   |-- CMakeLists.txt
|   `-- include/Common/
|       |-- types/
|       |   |-- DroneTypes.h
|       |   |-- LidarTypes.h
|       |   |-- MapTypes.h
|       |   `-- MissionTypes.h
|       |-- IDroneMovement.h
|       |-- IGPS.h
|       |-- ILidar.h
|       |-- IMap3D.h
|       |-- IMappingAlgorithm.h
|       |-- IMissionControl.h
|       |-- IMutableMap3D.h
|       |-- MappingAlgorithmFactory.h
|       |-- MappingAlgorithmRegistration.h
|       |-- MissionControlFactory.h
|       |-- MissionControlRegistration.h
|       |-- Types.h
|       `-- Units.h
|-- .gitignore
|-- CMakeLists.txt
|-- CMakePresets.json
|-- README.md
|-- students.txt
|-- vcpkg-configuration.json
`-- vcpkg.json
```

## Multithreading implementation notes

`Simulator/src/SimulatorRunner.cpp`'s `runComparative`/`runCompetition` distribute work across
`num_threads` worker threads (in addition to the main thread) when requested:

- **Unit of concurrency:** one whole *component* — one `MissionControl_*.so` in comparative mode,
  one `Algorithm_*.so` in competition mode. The composition sweep (simulation/mission/drone/lidar
  combinations) inside a single component's run stays sequential; only the top-level
  per-component loop is parallelized.
- **Worker count (`computeWorkerCount`):** `0` if `num_threads` is absent/`<= 1`, or if there's
  only one component to process; otherwise `min(num_threads, component_count)`. The `<= 1`
  component case is folded into the `0` branch specifically so the total thread count is never
  exactly `2` (1 main + 1 worker) — the assignment's stated invariant — since a single component
  has no other work to overlap with anyway.
- **Work distribution:** a shared `std::atomic<std::size_t>` cursor that every worker thread
  claims the next component index from via `fetch_add`, rather than a static up-front split. This
  keeps threads busy even when components take very different amounts of wall-clock time (e.g. one
  `.so` fails to load instantly while another runs a full sweep).
- **Locking:** this plan introduces exactly one lock — the `write_mutex_` inside
  `CerrSinkGuard::ConcurrentContextStreambuf` (`Simulator/src/CerrContextGuard.cpp`), guarding the
  final `sputn` of one already-assembled, prefixed `error.log` line so concurrent components' log
  lines are never interleaved mid-line. Per-thread line accumulation itself is lock-free
  (`thread_local`). The only other lock in the codebase is the pre-existing
  `Registrar::load_mutex_`, guarding `dlopen`/registration bookkeeping across concurrent
  `loadMappingAlgorithm`/`loadMissionControl` calls. `results_summary`/`errors` ordering needs no
  lock at all: each worker writes only to its own preallocated `outcomes[index]` slot, and the
  final report is rebuilt from those slots in order after every worker has joined.
