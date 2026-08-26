# Manual E2E YAML Pack

Place this `manual_e2e` directory directly under the repository's existing `inputs/` directory.

## Compositions
1. `01_staff_small.yaml` — staff Small scenarios, 2 staff drones x 2 staff LiDARs.
2. `02_staff_big.yaml` — staff Big scenarios, 2 staff drones x 2 staff LiDARs.
3. `03_staff_house.yaml` — staff House lower/full missions, 2 staff drones x 2 staff LiDARs.
4. `04_offset_equivalence.yaml` — three staff-vs-shifted pairs. Each pair maps to the same physical NPY start and same physical NPY bounds.
5. `05_custom_success_small.yaml` — two new starts x two new drones x two new LiDARs, generous step caps.
6. `06_custom_success_big.yaml` — same idea for Big.
7. `07_custom_success_house.yaml` — same idea for House.
8. `08_custom_bounds_offsets.yaml` — new mixed-sign offsets + genuinely new physical mission subsections, with known-good physical starts.
9. `09_zmax_measure_big.yaml` — controlled ZMAX measurement: same map/start/mission/drone; custom LiDARs differ only in z_max (180 vs 60).

## Important
- `10_zmax_boundary_big.yaml` is intentionally NOT included yet. Create it only after running #09 and observing the actual step counts. Then choose one shared `max_steps` between the two observed requirements.
- Files named `simulation/staff_*.yaml` reproduce the staff simulation values. Only `map_filename` was adapted because these copies live under `inputs/manual_e2e/simulation/`.
- Custom-success start points are intended candidate free-space starts selected for the serious maps. The manual runs themselves are the final E2E validation; if a start unexpectedly collides, do not "fix" the algorithm around it—inspect the specific geometry first.
- The controlled offset convention is: `map_local = world + offset`.
