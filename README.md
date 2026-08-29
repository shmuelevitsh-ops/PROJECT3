# Assignment 3 - Drone Mapper Simulator

Advanced Topics in Programming, Semester B 2026, Tel Aviv University.

This repository implements Assignment 3: a multithreaded **Simulator** that
dynamically loads `Algorithm` and `MissionControl` plugins (built as shared
libraries) and runs them, in either **comparative** mode (one Algorithm vs.
several MissionControls) or **competitive** mode (one MissionControl vs.
several Algorithms), across a configurable number of worker threads.

## Authors

| Name              | ID        |
|-------------------|-----------|
| Shireal Adler     | 322889890 |
| Adi Shmuelevitsh  | 315113738 |

(see [students.txt](students.txt))


## Building

```bash
cmake --preset default
cmake --build --preset default
```

This builds all three targets into `build/default/bin/`:

```text
simulator_322889890_315113738              # Simulator executable
Algorithm_322889890_315113738.so           # Algorithm plugin
MissionControl_322889890_315113738.so      # MissionControl plugin
```


## Running

The simulator has two mutually exclusive run modes. All arguments are
`key=value` (no spaces around `=`), may appear in any order, and are all
mandatory unless marked optional.

**Comparative run** - one Algorithm against every MissionControl `.so`
directly under a folder:

```bash
./simulator_322889890_315113738 -comparative \
    simulation=<simulation_compositions.yaml> \
    mission_control_folder=<folder_of_mission_control_so_files> \
    algorithm=<algorithm.so> \
    [num_threads=<N>] [-verbose]
```

**Competition run** - one MissionControl against every Algorithm `.so`
directly under a folder:

```bash
./simulator_322889890_315113738 -competition \
    simulation=<simulation_compositions.yaml> \
    mission_control=<mission_control.so> \
    algorithms_folder=<folder_of_algorithm_so_files> \
    [num_threads=<N>] [-verbose]
```

`num_threads` is optional: omitted or `1` runs everything on the main
thread; `N >= 2` uses up to `N` additional worker threads (so the process
never runs with exactly 2 threads total). `-verbose` makes MissionControl
write extra diagnostic output files for each run.


## Project Structure

```text
Simulator/          Simulator project - CLI parsing, plugin loading, threading,
                     result aggregation, and the simulator_322889890_315113738 executable
Algorithm/           Algorithm project - Algorithm_322889890_315113738.so
                     (namespace algorithm_322889890_315113738)
MissionControl/       MissionControl project - MissionControl_322889890_315113738.so
                     (namespace mission_control_322889890_315113738)
common/              Course-staff-published headers, used as-is (interfaces,
                     data types, factories, registration macros)
UserCommon/          Our own code shared between the three projects
                     (namespace user_common_322889890_315113738)
.devcontainer/       Dev container definition (Docker image with the toolchain)
CMakeLists.txt       Root CMake file - builds all three projects together
CMakePresets.json    "default" configure/build preset (Ninja + vcpkg)
vcpkg.json           Dependency manifest (mp-units, yaml-cpp, tinynpy, gtest)
students.txt         Submitter names and IDs
```

Each of `Simulator/`, `Algorithm/` and `MissionControl/` has its own
`CMakeLists.txt` and can be configured/built **standalone** (e.g. to compile
just an `Algorithm` `.so` against another team's `Simulator`), or all
together via the root `CMakeLists.txt`. `common/` and `UserCommon/` have no
makefiles of their own, per the assignment: `common/` contains only the
files published by the course staff, unmodified; `UserCommon/` contains our
own code that is shared by more than one project.


### Output

Each run creates a fresh, uniquely-named results directory directly under
the provided `mission_control_folder` (comparative) or `algorithms_folder`
(competition) - `comparative_results_<timestamp>` /
`competition_<timestamp>` - suffixed with `_2`, `_3`, ... on collision. It
contains:

- `error.log` - all error output produced during the run.
- One subfolder per component (Algorithm or MissionControl `.so`, named by
  its filename stem), containing the output map files and per-run artifacts
  for every simulation × mission × drone × lidar combination in the
  composition file, plus that component's own
  `simulation_output_<component>.yaml` (same per-run report format as
  Assignment 2).
- A single top-level `comparative_report.yaml` / `competitive_report.yaml`
  summarizing and ranking all components (see below), including any
  component that failed to load or run in a separate `errors` list.