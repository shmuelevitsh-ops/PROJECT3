# Mapping Algorithm Redesign — Implementation Brief

## Hard constraint

**Do not modify anything under `Common/`.**  
Keep all public interfaces and Common behavior unchanged. Implement the redesign only in the algorithm-side code and algorithm tests where needed.

---

## Problem in the current algorithm

The current implementation mixes a global logical Sweep cursor with generic Frontier detours.

When Sweep encounters a cell it cannot enter directly, it may advance `sweep_candidate` logically and immediately call the generic Frontier logic. That Frontier search is not tied to the Sweep candidate that caused the detour; it may move the drone to an unrelated reachable frontier and scan an unrelated unresolved voxel.

This creates a disconnect:

- the drone is physically moved somewhere else;
- `sweep_candidate` continues advancing according to the global boustrophedon order;
- Sweep can later resume with a candidate that is not physically adjacent to the drone;
- the logical Sweep may continue through cells and even through regions the drone never physically reached.

In enclosed or obstacle-heavy maps, the algorithm may therefore “sweep” large parts of the configured volume only by advancing the logical cursor, while the drone stays in a different reachable region.

The redesign should remove this disconnect.

---

## Desired high-level behavior

The algorithm should alternate between two behaviors:

`Local Sweep -> Frontier relocation -> Local Sweep -> Frontier relocation -> ...`

A Local Sweep is a real physical lawnmower traversal starting from the drone's current reachable location.

Frontier is used only when that local lawnmower traversal can no longer continue. Its purpose is to find a new useful reachable area, expose new free space, and provide a seed for a new Local Sweep.

There should no longer be generic Frontier detours in the middle of an active local Sweep.

---

## 1. Local Sweep behavior

Refactor Sweep so that it is based on the drone's real physical position rather than on a global cursor advancing through unreachable cells.

Use a local boustrophedon/lawnmower pattern:

- Sweep primarily along X.
- When the current X lane ends, try to move to the next Y lane and reverse the X direction.
- When no valid Y-lane continuation exists, try to continue on the next Z layer.
- When there is no valid continuation in X, Y, or Z, the current Local Sweep is exhausted.

A lane must end when either:

- the next cell reaches the configured map boundary;
- the next cell is confirmed `Occupied` or cannot be entered safely; or
- the next cell has already been marked `swept` by a previous Local Sweep.

A cell that is `Unmapped` or `PotentiallyOccupied` is **not** automatically an obstacle. If it can be scanned from the current state, scan it first. After the map is updated:

- if it becomes `Empty` and safe, it may be entered;
- if it becomes `Occupied` or unsafe, treat it as the end of the current lane.

Do not advance a Sweep cursor through cells the drone did not physically reach.

Obstacle handling should therefore behave like a local wall:

```text
→ → → → █
        ↓
← ← ← ←
```

while map boundaries and previously `swept` voxels behave the same way as lane-ending constraints for Local Sweep.

Before making the Y turn or moving to a new Z layer, apply the same occupancy, bounds, safety, and `swept` rules to the transition cell. If the transition cannot be performed safely or the transition cell was already `swept`, do not force it; try the next valid continuation according to the local Sweep logic, and if none exists, exhaust the Local Sweep.

---

## 2. Preserve Sweep batching

The current batching optimization must be preserved.

When the drone is moving along a Sweep lane and several consecutive cells are already known `Empty`, safe, and not previously `swept`, plan the whole consecutive run together instead of handling only the first candidate.

The batch must stop before the first voxel that is already `swept`, blocked, unsafe, unresolved in a way that requires a scan, or outside the valid lane.

For example, if five consecutive cells can be traversed safely:

```text
D E E E E E
```

the algorithm should plan all five as one continuous Sweep run.

Use the existing movement-building behavior so consecutive movement in the same direction is merged. Respect the drone command limits:

- `max_advance`
- `max_elevate`
- `max_rotate`

If the entire run fits inside one command, emit one command. If it exceeds a command limit, split only as required by that limit.

Keep the existing pipelined-scan optimization where appropriate: when the final movement command of a Sweep batch ends next to a new unresolved cell that can be scanned safely, attach the scan to that final movement command instead of adding an unnecessary separate step.

Do not regress this optimization.

---

## 3. Remove generic Frontier detours during an active Sweep

The current behavior where a blocked Sweep candidate falls through into the generic `frontierStep()` should be removed.

During a Local Sweep:

- do not search for an arbitrary unresolved target elsewhere in the map;
- do not move the drone to an unrelated frontier;
- do not keep a disconnected global `sweep_candidate`.

If the current lane is blocked, perform the local lawnmower turn logic.

If the local lawnmower traversal has no safe continuation in X/Y/Z, end that Local Sweep and only then enter Frontier relocation.

This should make Sweep physically coherent: while in Sweep mode, every planned movement belongs to the current local lawnmower traversal.

---

## 4. Frontier becomes a relocation/discovery mechanism

After a Local Sweep is exhausted, use Frontier/BFS to find a new useful mapping opportunity.

Keep the current core Frontier ideas:

- BFS only through reachable voxels that are known `Empty` and safe.
- A frontier/vantage point is a reachable safe voxel from which an unresolved target can be scanned.
- Respect LiDAR range and line-of-sight constraints.
- Preserve protection against repeating ineffective `(frontier, target)` attempts.

However, Frontier should not simply move to a vantage point and restart Sweep from that same old free voxel.

Its goal is to expose **new reachable free space**.

The expected flow is:

1. Find a reachable frontier/vantage point.
2. Move to it using BFS.
3. Scan an `Unmapped` / `PotentiallyOccupied` target.
4. Resolve the result on the following step.
5. If the target is `Occupied` or unsafe:
   - record the failed/finished attempt;
   - continue Frontier search.
6. If the scan exposes a new `Empty` voxel that is safe and reachable:
   - use that newly discovered free voxel as the seed for the next Local Sweep;
   - move to it safely if needed;
   - reset only the local Sweep traversal state;
   - transition back to Sweep mode.

The new Sweep seed should represent actual newly discovered reachable free space, not merely any previously known frontier voxel.

Frontier movement must continue using the existing movement-plan optimization: consecutive BFS path segments in the same direction should be merged and split only according to movement-command limits.

---

## 5. Repeated Sweep/Frontier cycles and `swept` tracking

The state machine should support repeated cycles:

```text
Local Sweep
   ↓ exhausted
Frontier
   ↓ new reachable free seed
Local Sweep
   ↓ exhausted
Frontier
   ↓
...
```

Maintain a global set of voxels that have already been physically traversed as part of a Local Sweep, for example `swept_voxels`.

A voxel is marked `swept` only when the drone physically enters/traverses it as part of Local Sweep movement. If a Sweep batch traverses several voxels, every voxel in that batch must be marked `swept`.

Voxels traversed only as part of Frontier/BFS relocation must **not** be marked `swept`.

During every Local Sweep movement decision, `swept` must be treated as an additional lane-ending condition:

- if the next Sweep voxel is not `swept`, continue with the normal occupancy, scan, safety, bounds, and batching logic;
- if the next Sweep voxel is already `swept`, do not enter it as part of the Local Sweep and do not re-sweep that already covered direction;
- instead, treat it like a local Sweep barrier, just as an obstacle or map boundary ends the current lane;
- then try the normal lawnmower continuation: move to the next valid Y lane and reverse X, or move to the next valid Z layer;
- only if no valid local continuation remains should the Local Sweep be considered exhausted and transition to Frontier.

A `swept` voxel is **not** globally blocked. It remains a valid known `Empty` voxel for Frontier/BFS paths and other safe relocation movement. The marker only prevents Local Sweep from covering the same voxel again.

When Frontier discovers a new reachable `Empty` seed and transitions back to Sweep, begin a fresh local lawnmower traversal from the drone's new physical position while preserving the global `swept_voxels` set and all Frontier progress such as attempted `(frontier, target)` pairs.

This ensures that repeated `Frontier -> Local Sweep` cycles expand into unswept space instead of repeatedly sweeping previously covered paths.

---

## 6. Safety rules must remain intact

Do not weaken any existing safety behavior.

All movement and batching must continue to respect:

- map bounds;
- voxel occupancy;
- `isSafeVoxel` / drone-radius collision clearance;
- safe reachable BFS paths;
- LiDAR range;
- line of sight;
- movement command limits;
- any existing collision checks used by the algorithm.

The drone must never enter an `Unmapped`, `PotentiallyOccupied`, `Occupied`, out-of-bounds, or otherwise unsafe voxel merely to preserve the lawnmower pattern.

If the desired lawnmower continuation is unsafe, the Sweep must turn, try another legal local continuation, or end.

---

## 7. Termination

After a Local Sweep is exhausted, Frontier should continue looking for a useful new mapping opportunity.

The algorithm should terminate only when Frontier cannot produce any additional useful progress.

Return:

- `Finished` when no unresolved mappable voxels remain.
- `FinishedWithUnmappableVoxels` when unresolved voxels remain but there is no reachable safe frontier that can expose additional reachable free space or otherwise make useful mapping progress.

---

## 8. Tests

Update/add algorithm tests to validate the redesigned behavior.

Some current tests may encode the old implementation detail that a blocked Sweep candidate immediately falls through to generic Frontier behavior. Update those tests only where they specifically depend on the old implementation. Do not weaken requirement-level or safety tests just to make the suite pass.

---

## Final implementation constraints

- **Do not modify `Common/`.**
- Keep public interfaces unchanged.
- Preserve the existing movement batching/merging optimization.
- Preserve pipelined scans where they remain valid.
- Preserve all safety and collision checks.
- Prefer adapting the existing Sweep, BFS, movement-queue, and Frontier helpers instead of rewriting unrelated parts of the algorithm.
- Keep the redesign focused on removing the logical-Sweep/physical-drone disconnect and implementing the repeated `Local Sweep -> Frontier -> Local Sweep` flow described above.
