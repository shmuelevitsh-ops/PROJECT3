#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/IMap3D.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

namespace Algorithm_322889890_315113738 {

namespace common_types = common::types;

using common::AltitudeAngle;
using common::HorizontalAngle;
using common::IMap3D;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::altitude_angle;
using common::cm;
using common::deg;
using common::horizontal_angle;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

constexpr double kAngleEpsilonDeg = 1e-6;
constexpr double kDistanceEpsilonCm = 1e-6;
// A target voxel is given at most this many scan attempts during the Sweep phase
// before the algorithm pivots away from it and leaves it for the Frontier phase.
constexpr int kMaxSweepScanAttempts = 1;

// Discrete map-grid coordinates used for all bookkeeping, BFS, and safety checks.
// Kept private to this translation unit (see MappingAlgorithmImpl.h pImpl comment).
struct VoxelIndex {
    long ix = 0;
    long iy = 0;
    long iz = 0;

    [[nodiscard]] bool operator==(const VoxelIndex& other) const {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

struct VoxelIndexHash {
    [[nodiscard]] std::size_t operator()(const VoxelIndex& idx) const {
        std::size_t seed = std::hash<long>{}(idx.ix);
        seed ^= std::hash<long>{}(idx.iy) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        seed ^= std::hash<long>{}(idx.iz) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct FrontierTargetPair {
    VoxelIndex frontier;
    VoxelIndex target;

    [[nodiscard]] bool operator==(const FrontierTargetPair& other) const {
        return frontier == other.frontier && target == other.target;
    }
};

struct FrontierTargetPairHash {
    [[nodiscard]] std::size_t operator()(const FrontierTargetPair& pair) const {
        const std::size_t h1 = VoxelIndexHash{}(pair.frontier);
        const std::size_t h2 = VoxelIndexHash{}(pair.target);
        return h1 ^ (h2 + 0x9e3779b9U + (h1 << 6) + (h1 >> 2));
    }
};

enum class Phase { Sweep, Frontier, Done };

// Bundles the algorithm's read-only dependencies so free helper functions don't need
// four separate parameters.
struct Context {
    const common_types::MissionConfigData& mission;
    const common_types::LidarConfigData& lidar;
    const common_types::DroneConfigData& drone;
    const IMap3D& map;
};

struct BfsResult {
    bool found = false;
    VoxelIndex frontier{};
    VoxelIndex target{};
    std::vector<VoxelIndex> path{}; // grid steps from the start position to `frontier`, exclusive of start.
};

[[nodiscard]] double numCm(PhysicalLength v) { return v.force_numerical_value_in(cm); }
[[nodiscard]] double numDeg(HorizontalAngle v) { return v.force_numerical_value_in(deg); }
[[nodiscard]] double numDeg(AltitudeAngle v) { return v.force_numerical_value_in(deg); }

// Wraps an angle in degrees to (-180, 180].
[[nodiscard]] double normalizeDeg(double angle_deg) {
    double wrapped = std::fmod(angle_deg + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped - 180.0;
}

[[nodiscard]] VoxelIndex toVoxelIndex(const Position3D& pos, const common_types::MapConfig& config) {
    const double resolution_cm = numCm(config.resolution);
    return VoxelIndex{
        static_cast<long>(std::floor((pos.x - config.offset.x).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.y - config.offset.y).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.z - config.offset.z).force_numerical_value_in(cm) / resolution_cm)),
    };
}

[[nodiscard]] Position3D toWorldCenter(const VoxelIndex& idx, const common_types::MapConfig& config) {
    const double resolution_cm = numCm(config.resolution);
    return Position3D{
        config.offset.x + (static_cast<double>(idx.ix) + 0.5) * resolution_cm * x_extent[cm],
        config.offset.y + (static_cast<double>(idx.iy) + 0.5) * resolution_cm * y_extent[cm],
        config.offset.z + (static_cast<double>(idx.iz) + 0.5) * resolution_cm * z_extent[cm],
    };
}

[[nodiscard]] bool isTargetOccupancy(common_types::VoxelOccupancy occ) {
    return occ == common_types::VoxelOccupancy::Unmapped || occ == common_types::VoxelOccupancy::PotentiallyOccupied;
}

[[nodiscard]] double voxelDistanceCm(const VoxelIndex& a, const VoxelIndex& b, double resolution_cm) {
    const double dx = static_cast<double>(a.ix - b.ix) * resolution_cm;
    const double dy = static_cast<double>(a.iy - b.iy) * resolution_cm;
    const double dz = static_cast<double>(a.iz - b.iz) * resolution_cm;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Rejects a target that is provably blocked by an already-discovered wall: a target within
// lidar range by straight-line distance can still be in a different room, with an Occupied
// voxel sitting directly between the frontier and it. Scanning toward such a target wastes a
// step (the beam hits the known wall, not the intended target) and burns the (frontier, target)
// pair as "tried" for no benefit. Walks the straight line at a fine sub-voxel step (matching
// ScanResultToVoxels' sampling granularity) so a thin diagonal wall is not skipped over.
// Unmapped/PotentiallyOccupied cells along the way are not treated as blocking — only a
// *known* Occupied voxel is a certain obstruction; anything else is still worth attempting.
[[nodiscard]] bool hasLineOfSight(const Context& ctx, const VoxelIndex& from, const VoxelIndex& to) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const double resolution_cm = numCm(config.resolution);
    const Position3D from_center = toWorldCenter(from, config);
    const Position3D to_center = toWorldCenter(to, config);
    const double dx = (to_center.x - from_center.x).force_numerical_value_in(cm);
    const double dy = (to_center.y - from_center.y).force_numerical_value_in(cm);
    const double dz = (to_center.z - from_center.z).force_numerical_value_in(cm);
    const double total_distance_cm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(resolution_cm > 0.0) || total_distance_cm <= kDistanceEpsilonCm) {
        return true;
    }

    // Coarser than ScanResultToVoxels' 0.1*resolution sampling: this is a pre-filter deciding
    // whether a scan is worth attempting, not the physical beam simulation itself (MockLidar
    // still does the precise hit test later) — half-voxel steps are enough to catch a wall
    // without paying for 10x as many atVoxel() calls per candidate.
    const double step_cm = 0.5 * resolution_cm;
    const long num_steps = static_cast<long>(std::floor(total_distance_cm / step_cm));
    for (long i = 1; i < num_steps; ++i) {
        const double t = (static_cast<double>(i) * step_cm) / total_distance_cm;
        const Position3D point{
            from_center.x + (t * dx) * x_extent[cm],
            from_center.y + (t * dy) * y_extent[cm],
            from_center.z + (t * dz) * z_extent[cm],
        };
        if (ctx.map.atVoxel(point) == common_types::VoxelOccupancy::Occupied) {
            return false;
        }
    }
    return true;
}

// Safety rule: every voxel overlapping the drone's safety sphere (radius drone_config_.radius,
// centered at the candidate's world center) must be VoxelOccupancy::Empty, and the candidate
// itself must be within the map's configured boundaries.
[[nodiscard]] bool isSafeVoxel(const Context& ctx, const VoxelIndex& idx) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const Position3D center = toWorldCenter(idx, config);
    if (!ctx.map.isInBounds(center)) {
        return false;
    }
    const double resolution_cm = numCm(config.resolution);
    if (!(resolution_cm > 0.0)) {
        return false;
    }
    const double radius_cm = numCm(ctx.drone.radius);
    const long voxel_radius = static_cast<long>(std::ceil(radius_cm / resolution_cm));

    for (long dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (long dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (long dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                const double offset_distance_cm =
                    std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_cm;
                if (offset_distance_cm > radius_cm) {
                    continue; // Voxel does not overlap the safety sphere.
                }
                const VoxelIndex neighbor{idx.ix + dx, idx.iy + dy, idx.iz + dz};
                if (ctx.map.atVoxel(toWorldCenter(neighbor, config)) != common_types::VoxelOccupancy::Empty) {
                    return false;
                }
            }
        }
    }
    return true;
}

// Computes the scan orientation relative to `heading` that points from `from` toward `to`,
// matching MockLidar::scan(), which adds the sensor heading to the supplied orientation.
[[nodiscard]] Orientation relativeScanOrientation(const Position3D& from, const Position3D& to,
                                                   const Orientation& heading) {
    const double dx = (to.x - from.x).force_numerical_value_in(cm);
    const double dy = (to.y - from.y).force_numerical_value_in(cm);
    const double dz = (to.z - from.z).force_numerical_value_in(cm);
    const double horizontal_dist_cm = std::sqrt(dx * dx + dy * dy);

    const double world_horizontal_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    const double world_altitude_deg = std::atan2(dz, horizontal_dist_cm) * 180.0 / M_PI;

    const double relative_horizontal_deg = normalizeDeg(world_horizontal_deg - numDeg(heading.horizontal));
    const double relative_altitude_deg = normalizeDeg(world_altitude_deg - numDeg(heading.altitude));

    return Orientation{relative_horizontal_deg * horizontal_angle[deg],
                        relative_altitude_deg * altitude_angle[deg]};
}

// Appends Rotate commands summing to `signed_angle_deg`, each within `max_rotate_deg`.
void pushChunkedRotate(std::deque<common_types::MovementCommand>& queue, double signed_angle_deg,
                        double max_rotate_deg) {
    double remaining_abs = std::fabs(signed_angle_deg);
    if (remaining_abs <= kAngleEpsilonDeg) {
        return;
    }
    const common_types::RotationDirection direction =
        signed_angle_deg >= 0.0 ? common_types::RotationDirection::Left : common_types::RotationDirection::Right;
    if (!(max_rotate_deg > 0.0)) {
        // Defensive: a non-positive limit cannot be chunked meaningfully; issue one best-effort command.
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Rotate;
        cmd.rotation = direction;
        cmd.angle = remaining_abs * horizontal_angle[deg];
        queue.push_back(cmd);
        return;
    }
    while (remaining_abs > kAngleEpsilonDeg) {
        const double step = std::min(remaining_abs, max_rotate_deg);
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Rotate;
        cmd.rotation = direction;
        cmd.angle = step * horizontal_angle[deg];
        queue.push_back(cmd);
        remaining_abs -= step;
    }
}

// Appends Advance commands summing to `distance_cm` (always forward, i.e. non-negative),
// each within `max_advance_cm`.
void pushChunkedAdvance(std::deque<common_types::MovementCommand>& queue, double distance_cm,
                         double max_advance_cm) {
    double remaining = distance_cm;
    if (remaining <= kDistanceEpsilonCm) {
        return;
    }
    if (!(max_advance_cm > 0.0)) {
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Advance;
        cmd.distance = remaining * cm;
        queue.push_back(cmd);
        return;
    }
    while (remaining > kDistanceEpsilonCm) {
        const double step = std::min(remaining, max_advance_cm);
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Advance;
        cmd.distance = step * cm;
        queue.push_back(cmd);
        remaining -= step;
    }
}

// Appends Elevate commands summing to `signed_distance_cm`, each within `max_elevate_cm`.
void pushChunkedElevate(std::deque<common_types::MovementCommand>& queue, double signed_distance_cm,
                         double max_elevate_cm) {
    double remaining_abs = std::fabs(signed_distance_cm);
    if (remaining_abs <= kDistanceEpsilonCm) {
        return;
    }
    const double sign = signed_distance_cm >= 0.0 ? 1.0 : -1.0;
    if (!(max_elevate_cm > 0.0)) {
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Elevate;
        cmd.distance = (sign * remaining_abs) * cm;
        queue.push_back(cmd);
        return;
    }
    while (remaining_abs > kDistanceEpsilonCm) {
        const double step = std::min(remaining_abs, max_elevate_cm);
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Elevate;
        cmd.distance = (sign * step) * cm;
        queue.push_back(cmd);
        remaining_abs -= step;
    }
}

// Converts a sequence of single-voxel grid steps (each differing from its predecessor by
// exactly one resolution step along one axis) into a queue of bounded MovementCommands,
// rotating to face each horizontal step before advancing into it.
[[nodiscard]] std::deque<common_types::MovementCommand> buildMovementQueue(const Context& ctx,
                                                                     const Orientation& start_heading,
                                                                     const VoxelIndex& start,
                                                                     const std::vector<VoxelIndex>& path) {
    std::deque<common_types::MovementCommand> queue;
    const double resolution_cm = numCm(ctx.map.getMapConfig().resolution);
    const double max_rotate_deg = numDeg(ctx.drone.max_rotate);
    const double max_advance_cm = numCm(ctx.drone.max_advance);
    const double max_elevate_cm = numCm(ctx.drone.max_elevate);

    double heading_deg = numDeg(start_heading.horizontal);
    VoxelIndex prev = start;
    for (const VoxelIndex& step : path) {
        const long dx = step.ix - prev.ix;
        const long dy = step.iy - prev.iy;
        const long dz = step.iz - prev.iz;

        if (dz != 0) {
            pushChunkedElevate(queue, static_cast<double>(dz) * resolution_cm, max_elevate_cm);
        }
        if (dx != 0 || dy != 0) {
            const double target_heading_deg =
                std::atan2(static_cast<double>(dy), static_cast<double>(dx)) * 180.0 / M_PI;
            const double diff_deg = normalizeDeg(target_heading_deg - heading_deg);
            if (std::fabs(diff_deg) > kAngleEpsilonDeg) {
                pushChunkedRotate(queue, diff_deg, max_rotate_deg);
                heading_deg = normalizeDeg(heading_deg + diff_deg);
            }
            const double step_distance_cm = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_cm;
            pushChunkedAdvance(queue, step_distance_cm, max_advance_cm);
        }
        prev = step;
    }
    return queue;
}

[[nodiscard]] common_types::MappingStepCommand statusOnlyCommand(common_types::AlgorithmStatus status) {
    common_types::MappingStepCommand result;
    result.status = status;
    return result;
}

[[nodiscard]] common_types::MappingStepCommand movementOnlyCommand(const common_types::MovementCommand& movement) {
    common_types::MappingStepCommand result;
    result.movement = movement;
    result.status = common_types::AlgorithmStatus::Working;
    return result;
}

} // namespace

// Per-instance state that must persist across nextStep() calls. See the pImpl comment in
// MappingAlgorithmImpl.h for why this lives here rather than as header-visible members.
struct MappingAlgorithmImpl::Impl {
    Phase phase = Phase::Sweep;
    std::deque<common_types::MovementCommand> pending_moves{};

    // Frontier bookkeeping: unresolved targets plus the frontier/target pairs
    // already attempted from a specific vantage point.
    std::unordered_set<VoxelIndex, VoxelIndexHash> unresolved_targets{};
    std::unordered_set<FrontierTargetPair, FrontierTargetPairHash> tried_pairs{};

    // Sweep-phase lawnmower cursor.
    bool sweep_initialized = false;
    VoxelIndex sweep_candidate{};
    int sweep_dir_x = 1;
    long sweep_min_x = 0, sweep_max_x = 0;
    long sweep_min_y = 0, sweep_max_y = 0;
    long sweep_min_z = 0, sweep_max_z = 0;
    int sweep_repeat_scan_count = 0;

    // Tracks the scan most recently requested, so the *next* call can tell whether it resolved.
    std::optional<VoxelIndex> pending_scan_target{};
    std::optional<VoxelIndex> pending_scan_frontier{};
    bool pending_scan_in_frontier_phase = false;

    // The frontier the drone is currently travelling toward, and the target it intends to
    // scan upon arrival (Frontier phase only).
    std::optional<VoxelIndex> planned_frontier{};
    std::optional<VoxelIndex> planned_target{};

    common_types::AlgorithmStatus final_status = common_types::AlgorithmStatus::Working;

    common_types::MappingStepCommand nextStep(const Context& ctx, const common_types::DroneState& state,
                                        const common_types::LidarScanResult* latest_scan);

private:
    void resolvePendingScan(const Context& ctx);
    common_types::MappingStepCommand sweepStep(const Context& ctx, const common_types::DroneState& state);
    common_types::MappingStepCommand frontierStep(const Context& ctx, const common_types::DroneState& state);
    common_types::MappingStepCommand scanCommand(const Context& ctx, const common_types::DroneState& state,
                                          const VoxelIndex& target,
                                          const std::optional<VoxelIndex>& frontier);

    void initSweepBounds(const Context& ctx, const VoxelIndex& start);
    [[nodiscard]] bool advanceSweepCandidate();
    [[nodiscard]] common_types::MappingStepCommand popPendingMove();

    [[nodiscard]] std::optional<VoxelIndex> findUntriedTargetNear(const Context& ctx, const VoxelIndex& s);
    [[nodiscard]] BfsResult frontierBfs(const Context& ctx, const VoxelIndex& start);
    [[nodiscard]] bool hasAnyUnresolvedVoxel(const Context& ctx) const;
};

common_types::MappingStepCommand MappingAlgorithmImpl::Impl::nextStep(const Context& ctx,
                                                                 const common_types::DroneState& state,
                                                                 const common_types::LidarScanResult* latest_scan) {
    // output_map_ is the ground truth (already updated by DroneControlImpl before this call);
    // latest_scan is offered only for convenience and is not required for correct behavior.
    (void)latest_scan;

    resolvePendingScan(ctx);

    if (!pending_moves.empty()) {
        return popPendingMove();
    }

    if (phase == Phase::Sweep) {
        return sweepStep(ctx, state);
    }
    return frontierStep(ctx, state);
}

common_types::MappingStepCommand MappingAlgorithmImpl::Impl::popPendingMove() {
    const common_types::MovementCommand cmd = pending_moves.front();
    pending_moves.pop_front();
    return movementOnlyCommand(cmd);
}

void MappingAlgorithmImpl::Impl::resolvePendingScan(const Context& ctx) {
    if (!pending_scan_target) {
        return;
    }
    const Position3D center = toWorldCenter(*pending_scan_target, ctx.map.getMapConfig());
    const bool resolved = !isTargetOccupancy(ctx.map.atVoxel(center));

    if (pending_scan_in_frontier_phase && pending_scan_frontier) {
        if (resolved) {
            unresolved_targets.erase(*pending_scan_target);
        } else {
            tried_pairs.insert(FrontierTargetPair{*pending_scan_frontier, *pending_scan_target});
        }
    } else {
        sweep_repeat_scan_count = resolved ? 0 : sweep_repeat_scan_count + 1;
    }

    pending_scan_target.reset();
    pending_scan_frontier.reset();
    pending_scan_in_frontier_phase = false;
}

common_types::MappingStepCommand MappingAlgorithmImpl::Impl::scanCommand(const Context& ctx,
                                                                    const common_types::DroneState& state,
                                                                    const VoxelIndex& target,
                                                                    const std::optional<VoxelIndex>& frontier) {
    pending_scan_target = target;
    pending_scan_frontier = frontier;
    pending_scan_in_frontier_phase = frontier.has_value();

    const Position3D target_center = toWorldCenter(target, ctx.map.getMapConfig());
    common_types::MappingStepCommand result;
    result.scan_orientation = relativeScanOrientation(state.position, target_center, state.heading);
    result.status = common_types::AlgorithmStatus::Working;
    return result;
}

void MappingAlgorithmImpl::Impl::initSweepBounds(const Context& ctx, const VoxelIndex& start) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const VoxelIndex min_idx = toVoxelIndex(
        Position3D{config.boundaries.min_x, config.boundaries.min_y, config.boundaries.min_height}, config);
    // The upper boundary is exclusive; the last valid index is one below it.
    const VoxelIndex max_idx_exclusive = toVoxelIndex(
        Position3D{config.boundaries.max_x, config.boundaries.max_y, config.boundaries.max_height}, config);

    sweep_min_x = min_idx.ix;
    sweep_min_y = min_idx.iy;
    sweep_min_z = min_idx.iz;
    sweep_max_x = max_idx_exclusive.ix - 1;
    sweep_max_y = max_idx_exclusive.iy - 1;
    sweep_max_z = max_idx_exclusive.iz - 1;
    sweep_dir_x = 1;

    // The drone's own starting cell is assumed safe by definition and is never itself a
    // sweep target; begin from the next lawnmower cell.
    sweep_candidate = start;
    if (advanceSweepCandidate()) {
        phase = Phase::Frontier;
    }
    sweep_initialized = true;
}

bool MappingAlgorithmImpl::Impl::advanceSweepCandidate() {
    const long next_x = sweep_candidate.ix + sweep_dir_x;
    if (next_x >= sweep_min_x && next_x <= sweep_max_x) {
        sweep_candidate.ix = next_x;
        return false;
    }
    // Lane exhausted: pivot to the next row, keeping the current x coordinate and flipping
    // direction (boustrophedon pattern).
    sweep_dir_x = -sweep_dir_x;
    sweep_candidate.iy += 1;
    if (sweep_candidate.iy > sweep_max_y) {
        sweep_candidate.iy = sweep_min_y;
        sweep_candidate.iz += 1;
        if (sweep_candidate.iz > sweep_max_z) {
            return true; // Entire sweep volume exhausted.
        }
    }
    return false;
}

common_types::MappingStepCommand MappingAlgorithmImpl::Impl::sweepStep(const Context& ctx,
                                                                  const common_types::DroneState& state) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const VoxelIndex cur = toVoxelIndex(state.position, config);
    if (!sweep_initialized) {
        initSweepBounds(ctx, cur);
    }

    // The next sweep candidate is only ever moved into directly when it is exactly one
    // resolution step (face-adjacent) from `cur`. That holds for ordinary lane progress and for
    // a row-boundary pivot (see advanceSweepCandidate), but not for the rarer case where a full
    // xy-layer has just been completed and the cursor wraps back to the layer's starting row
    // while also stepping up to the next z-layer - that wrap is more than one voxel away from
    // `cur`. In every case where the candidate is not face-adjacent, or is face-adjacent but
    // blocked (Occupied / OutOfBounds / an Empty cell failing the sphere safety check, or a
    // target already scanned the maximum number of times without resolving), defer this one
    // step to the Frontier phase, which performs a properly step-validated BFS instead of a
    // single straight-line movement that would skip over unvalidated intermediate cells. This is
    // a one-step detour, not a permanent mode switch: `phase` stays Sweep (see below) so the
    // sweep cursor — now advanced past the cell that blocked it — gets retried on the next call,
    // instead of abandoning the systematic sweep for the rest of the mission over one obstacle.
    const VoxelIndex candidate = sweep_candidate;
    const long dx = candidate.ix - cur.ix;
    const long dy = candidate.iy - cur.iy;
    const long dz = candidate.iz - cur.iz;
    const bool candidate_is_face_adjacent = std::abs(dx) + std::abs(dy) + std::abs(dz) == 1;

    if (candidate_is_face_adjacent) {
        const common_types::VoxelOccupancy occ = ctx.map.atVoxel(toWorldCenter(candidate, config));

        if (isTargetOccupancy(occ) && sweep_repeat_scan_count < kMaxSweepScanAttempts) {
            return scanCommand(ctx, state, candidate, std::nullopt);
        }

        if (occ == common_types::VoxelOccupancy::Empty && isSafeVoxel(ctx, candidate)) {
            pending_moves = buildMovementQueue(ctx, state.heading, cur, {candidate});
            if (advanceSweepCandidate()) {
                phase = Phase::Frontier;
            }
            if (!pending_moves.empty()) {
                return popPendingMove();
            }
            return frontierStep(ctx, state);
        }
        sweep_repeat_scan_count = 0;
    }

    // Skip this candidate — Frontier's BFS will pick it up later if/when it becomes reachable
    // from elsewhere — and let one Frontier-driven action make safe progress now. Only a fully
    // exhausted sweep volume (cursor has walked off the end) is a genuine permanent transition.
    if (advanceSweepCandidate()) {
        phase = Phase::Frontier;
    }
    return frontierStep(ctx, state);
}

std::optional<VoxelIndex> MappingAlgorithmImpl::Impl::findUntriedTargetNear(const Context& ctx,
                                                                              const VoxelIndex& s) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const double resolution_cm = numCm(config.resolution);
    if (!(resolution_cm > 0.0)) {
        return std::nullopt;
    }
    const double z_min_cm = numCm(ctx.lidar.z_min);
    const double z_max_cm = numCm(ctx.lidar.z_max);
    const long voxel_radius = static_cast<long>(std::ceil(z_max_cm / resolution_cm));

    std::optional<VoxelIndex> chosen;
    for (long dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (long dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (long dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                const VoxelIndex candidate{s.ix + dx, s.iy + dy, s.iz + dz};
                const double distance_cm = voxelDistanceCm(candidate, s, resolution_cm);
                if (distance_cm < z_min_cm || distance_cm > z_max_cm) {
                    continue;
                }
                const Position3D center = toWorldCenter(candidate, config);
                if (!ctx.map.isInBounds(center) || !isTargetOccupancy(ctx.map.atVoxel(center))) {
                    continue;
                }

                unresolved_targets.insert(candidate);
                if (!chosen && tried_pairs.count(FrontierTargetPair{s, candidate}) == 0 &&
                    hasLineOfSight(ctx, s, candidate)) {
                    chosen = candidate;
                }
            }
        }
    }
    return chosen;
}

BfsResult MappingAlgorithmImpl::Impl::frontierBfs(const Context& ctx, const VoxelIndex& start) {
    BfsResult result;
    std::queue<VoxelIndex> open;
    std::unordered_map<VoxelIndex, VoxelIndex, VoxelIndexHash> parent;
    std::unordered_set<VoxelIndex, VoxelIndexHash> visited;
    open.push(start);
    visited.insert(start);

    static constexpr std::array<VoxelIndex, 6> kFaceDirs{
        {VoxelIndex{1, 0, 0}, VoxelIndex{-1, 0, 0}, VoxelIndex{0, 1, 0}, VoxelIndex{0, -1, 0},
         VoxelIndex{0, 0, 1}, VoxelIndex{0, 0, -1}}};

    while (!open.empty()) {
        const VoxelIndex s = open.front();
        open.pop();

        if (const std::optional<VoxelIndex> target = findUntriedTargetNear(ctx, s)) {
            result.found = true;
            result.frontier = s;
            result.target = *target;
            for (VoxelIndex node = s; !(node == start); node = parent.at(node)) {
                result.path.push_back(node);
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (const VoxelIndex& d : kFaceDirs) {
            const VoxelIndex n{s.ix + d.ix, s.iy + d.iy, s.iz + d.iz};
            if (visited.count(n) != 0) {
                continue;
            }
            visited.insert(n);
            if (ctx.map.atVoxel(toWorldCenter(n, ctx.map.getMapConfig())) != common_types::VoxelOccupancy::Empty) {
                continue;
            }
            if (!isSafeVoxel(ctx, n)) {
                continue;
            }
            parent[n] = s;
            open.push(n);
        }
    }
    return result;
}

bool MappingAlgorithmImpl::Impl::hasAnyUnresolvedVoxel(const Context& ctx) const {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const double resolution_cm = numCm(config.resolution);
    if (!(resolution_cm > 0.0)) {
        return false;
    }
    const VoxelIndex min_idx = toVoxelIndex(
        Position3D{config.boundaries.min_x, config.boundaries.min_y, config.boundaries.min_height}, config);
    const VoxelIndex max_idx_exclusive = toVoxelIndex(
        Position3D{config.boundaries.max_x, config.boundaries.max_y, config.boundaries.max_height}, config);

    for (long ix = min_idx.ix; ix < max_idx_exclusive.ix; ++ix) {
        for (long iy = min_idx.iy; iy < max_idx_exclusive.iy; ++iy) {
            for (long iz = min_idx.iz; iz < max_idx_exclusive.iz; ++iz) {
                const Position3D center = toWorldCenter(VoxelIndex{ix, iy, iz}, config);
                if (ctx.map.isInBounds(center) && isTargetOccupancy(ctx.map.atVoxel(center))) {
                    return true;
                }
            }
        }
    }
    return false;
}

common_types::MappingStepCommand MappingAlgorithmImpl::Impl::frontierStep(const Context& ctx,
                                                                     const common_types::DroneState& state) {
    if (phase == Phase::Done) {
        return statusOnlyCommand(final_status);
    }
    // Deliberately does not set `phase = Phase::Frontier` here: this function is also called as
    // a one-step detour from sweepStep() while `phase` is still Sweep, and must not permanently
    // latch Frontier in that case (see sweepStep()'s comment). Genuine permanent transitions are
    // set by the caller (initSweepBounds() / sweepStep() on sweep-volume exhaustion) before
    // reaching here, or below in this function once the mission is truly Done.

    const VoxelIndex cur = toVoxelIndex(state.position, ctx.map.getMapConfig());

    if (planned_frontier && *planned_frontier == cur) {
        const VoxelIndex target = *planned_target;
        planned_frontier.reset();
        planned_target.reset();
        return scanCommand(ctx, state, target, cur);
    }

    const BfsResult bfs = frontierBfs(ctx, cur);
    if (!bfs.found) {
        // One-time fallback boundary scan, as explicitly allowed by the assignment spec, to
        // distinguish a fully-mapped map from unresolved voxels that incremental frontier
        // discovery never reached (e.g. fully enclosed pockets).
        final_status = hasAnyUnresolvedVoxel(ctx) ? common_types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                                   : common_types::AlgorithmStatus::Finished;
        phase = Phase::Done;
        return statusOnlyCommand(final_status);
    }

    if (bfs.frontier == cur) {
        return scanCommand(ctx, state, bfs.target, cur);
    }

    pending_moves = buildMovementQueue(ctx, state.heading, cur, bfs.path);
    if (pending_moves.empty()) {
        // Defensive: a non-trivial path with no resulting commands should not happen since
        // bfs.frontier != cur, but guard against an infinite stall regardless.
        final_status = common_types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        phase = Phase::Done;
        return statusOnlyCommand(final_status);
    }

    planned_frontier = bfs.frontier;
    planned_target = bfs.target;
    return popPendingMove();
}

MappingAlgorithmImpl::MappingAlgorithmImpl(
    common::MappingAlgorithmDependencies dependencies)
    : common::IMappingAlgorithm(std::move(dependencies)) {}

MappingAlgorithmImpl::~MappingAlgorithmImpl() = default;

common_types::MappingStepCommand MappingAlgorithmImpl::nextStep(const common_types::DroneState& state,
                                                          const common_types::LidarScanResult* latest_scan) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    const Context ctx{mission_config_, lidar_config_, drone_config_, output_map_};
    return impl_->nextStep(ctx, state, latest_scan);
}

REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl);

} // namespace Algorithm_322889890_315113738
