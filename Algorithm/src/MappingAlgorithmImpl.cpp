#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/IMap3D.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <UserCommon/SphereAabbCollision.h>

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

namespace algorithm_322889890_315113738 {

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
// Maximum Sweep scan attempts for one unresolved target.
constexpr int kMaxSweepScanAttempts = 1;
// Sampling step for the line-of-sight pre-filter.
constexpr double kLineOfSightSampleStepFraction = 0.5;

// Discrete map-grid coordinates.
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

// Groups the algorithm's read-only dependencies.
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

// Movement commands and the resulting horizontal heading.
struct MovementPlan {
    std::deque<common_types::MovementCommand> commands{};
    double final_heading_deg = 0.0;
};

[[nodiscard]] double numCm(PhysicalLength v) { return v.force_numerical_value_in(cm); }
[[nodiscard]] double numDeg(HorizontalAngle v) { return v.force_numerical_value_in(deg); }
[[nodiscard]] double numDeg(AltitudeAngle v) { return v.force_numerical_value_in(deg); }

// atan2(y, x), converted to degrees.
[[nodiscard]] double atan2Deg(double y, double x) {
    return std::atan2(y, x) * 180.0 / M_PI;
}

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
        static_cast<long>(std::floor((pos.x + config.offset.x).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.y + config.offset.y).force_numerical_value_in(cm) / resolution_cm)),
        static_cast<long>(std::floor((pos.z + config.offset.z).force_numerical_value_in(cm) / resolution_cm)),
    };
}

[[nodiscard]] Position3D toWorldCenter(const VoxelIndex& idx, const common_types::MapConfig& config) {
    const double resolution_cm = numCm(config.resolution);
    return Position3D{
        (static_cast<double>(idx.ix) + 0.5) * resolution_cm * x_extent[cm] - config.offset.x,
        (static_cast<double>(idx.iy) + 0.5) * resolution_cm * y_extent[cm] - config.offset.y,
        (static_cast<double>(idx.iz) + 0.5) * resolution_cm * z_extent[cm] - config.offset.z,
    };
}

// Minimum corner of the voxel's axis-aligned bounding box.
[[nodiscard]] Position3D voxelMinCorner(const VoxelIndex& idx, const common_types::MapConfig& config) {
    const double resolution_cm = numCm(config.resolution);
    return Position3D{
        static_cast<double>(idx.ix) * resolution_cm * x_extent[cm] - config.offset.x,
        static_cast<double>(idx.iy) * resolution_cm * y_extent[cm] - config.offset.y,
        static_cast<double>(idx.iz) * resolution_cm * z_extent[cm] - config.offset.z,
    };
}

[[nodiscard]] Position3D voxelMaxCorner(const VoxelIndex& idx, const common_types::MapConfig& config) {
    const double resolution_cm = numCm(config.resolution);
    return Position3D{
        (static_cast<double>(idx.ix) + 1.0) * resolution_cm * x_extent[cm] - config.offset.x,
        (static_cast<double>(idx.iy) + 1.0) * resolution_cm * y_extent[cm] - config.offset.y,
        (static_cast<double>(idx.iz) + 1.0) * resolution_cm * z_extent[cm] - config.offset.z,
    };
}

// Preserves the drone's real offset from its voxel center during planning.
[[nodiscard]] Position3D intraVoxelOffset(const Context& ctx, const common_types::DroneState& state) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    return state.position - toWorldCenter(toVoxelIndex(state.position, config), config);
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

// True if `a` and `b` differ by exactly one resolution step along exactly one axis.
[[nodiscard]] bool isFaceAdjacent(const VoxelIndex& a, const VoxelIndex& b) {
    return std::abs(a.ix - b.ix) + std::abs(a.iy - b.iy) + std::abs(a.iz - b.iz) == 1;
}

// Preserves the drone's real offset from its voxel center during planning.
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

    // A coarse pre-filter; the LiDAR performs the precise hit test.
    const double step_cm = kLineOfSightSampleStepFraction * resolution_cm;
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

// A coarse pre-filter; the LiDAR performs the precise hit test.
[[nodiscard]] bool isSafeVoxel(const Context& ctx, const VoxelIndex& idx, const Position3D& offset) {
    const common_types::MapConfig config = ctx.map.getMapConfig();
    const Position3D center = toWorldCenter(idx, config) + offset;
    if (!ctx.map.isInBounds(center)) {
        return false;
    }
    const double resolution_cm = numCm(config.resolution);
    if (!(resolution_cm > 0.0)) {
        return false;
    }
    const double radius_cm = numCm(ctx.drone.radius);
    // Requires every voxel intersecting the drone's safety sphere to be Empty.
    const long voxel_radius = static_cast<long>(std::ceil(radius_cm / resolution_cm)) + 1;

    for (long dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (long dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (long dz = -voxel_radius; dz <= voxel_radius; ++dz) {
                const VoxelIndex neighbor{idx.ix + dx, idx.iy + dy, idx.iz + dz};
                if (!user_common_322889890_315113738::sphereIntersectsAxisAlignedBox(
                        center, ctx.drone.radius, voxelMinCorner(neighbor, config),
                        voxelMaxCorner(neighbor, config))) {
                    continue; // Voxel's volume does not overlap the safety sphere.
                }
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

    const double world_horizontal_deg = atan2Deg(dy, dx);
    const double world_altitude_deg = atan2Deg(dz, horizontal_dist_cm);

    const double relative_horizontal_deg = normalizeDeg(world_horizontal_deg - numDeg(heading.horizontal));
    const double relative_altitude_deg = normalizeDeg(world_altitude_deg - numDeg(heading.altitude));

    return Orientation{relative_horizontal_deg * horizontal_angle[deg],
                        relative_altitude_deg * altitude_angle[deg]};
}

// Splits a magnitude into pieces no larger than max_magnitude.
[[nodiscard]] std::vector<double> splitMagnitude(double total, double max_magnitude, double epsilon) {
    if (total <= epsilon) {
        return {};
    }
    if (!(max_magnitude > 0.0)) {
        return {total};
    }
    std::vector<double> pieces;
    double remaining = total;
    while (remaining > epsilon) {
        const double step = std::min(remaining, max_magnitude);
        pieces.push_back(step);
        remaining -= step;
    }
    return pieces;
}

// Appends Rotate commands summing to `signed_angle_deg`, each within `max_rotate_deg`.
void pushChunkedRotate(std::deque<common_types::MovementCommand>& queue, double signed_angle_deg,
                        double max_rotate_deg) {
    const std::vector<double> pieces =
        splitMagnitude(std::fabs(signed_angle_deg), max_rotate_deg, kAngleEpsilonDeg);
    if (pieces.empty()) {
        return;
    }
    const common_types::RotationDirection direction =
        signed_angle_deg >= 0.0 ? common_types::RotationDirection::Left : common_types::RotationDirection::Right;
    for (const double piece_deg : pieces) {
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Rotate;
        cmd.rotation = direction;
        cmd.angle = piece_deg * horizontal_angle[deg];
        queue.push_back(cmd);
    }
}

// Appends Advance commands summing to `distance_cm` (always forward, i.e. non-negative),
// each within `max_advance_cm`.
void pushChunkedAdvance(std::deque<common_types::MovementCommand>& queue, double distance_cm,
                         double max_advance_cm) {
    const std::vector<double> pieces = splitMagnitude(distance_cm, max_advance_cm, kDistanceEpsilonCm);
    for (const double piece_cm : pieces) {
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Advance;
        cmd.distance = piece_cm * cm;
        queue.push_back(cmd);
    }
}

// Appends Elevate commands summing to `signed_distance_cm`, each within `max_elevate_cm`.
void pushChunkedElevate(std::deque<common_types::MovementCommand>& queue, double signed_distance_cm,
                         double max_elevate_cm) {
    const std::vector<double> pieces =
        splitMagnitude(std::fabs(signed_distance_cm), max_elevate_cm, kDistanceEpsilonCm);
    if (pieces.empty()) {
        return;
    }
    const double sign = signed_distance_cm >= 0.0 ? 1.0 : -1.0;
    for (const double piece_cm : pieces) {
        common_types::MovementCommand cmd;
        cmd.type = common_types::MovementCommandType::Elevate;
        cmd.distance = (sign * piece_cm) * cm;
        queue.push_back(cmd);
    }
}

// Converts a validated voxel path into bounded movement commands, merging consecutive runs.
[[nodiscard]] MovementPlan buildMovementQueue(const Context& ctx, const Orientation& start_heading,
                                               const VoxelIndex& start, const std::vector<VoxelIndex>& path) {
    MovementPlan plan;
    const double resolution_cm = numCm(ctx.map.getMapConfig().resolution);
    const double max_rotate_deg = numDeg(ctx.drone.max_rotate);
    const double max_advance_cm = numCm(ctx.drone.max_advance);
    const double max_elevate_cm = numCm(ctx.drone.max_elevate);

    double heading_deg = numDeg(start_heading.horizontal);
    VoxelIndex prev = start;
    std::size_t i = 0;
    while (i < path.size()) {
        // Direction of the run starting at path[i], relative to its immediate predecessor.
        const long dir_x = path[i].ix - prev.ix;
        const long dir_y = path[i].iy - prev.iy;
        const long dir_z = path[i].iz - prev.iz;

        // Extend the run while subsequent steps continue in that exact same direction.
        std::size_t run_end = i + 1;
        VoxelIndex run_prev = path[i];
        while (run_end < path.size() && path[run_end].ix - run_prev.ix == dir_x &&
               path[run_end].iy - run_prev.iy == dir_y && path[run_end].iz - run_prev.iz == dir_z) {
            run_prev = path[run_end];
            ++run_end;
        }
        const auto run_length = static_cast<double>(run_end - i);

        if (dir_z != 0) {
            pushChunkedElevate(plan.commands, static_cast<double>(dir_z) * run_length * resolution_cm,
                                max_elevate_cm);
        } else if (dir_x != 0 || dir_y != 0) {
            const double target_heading_deg =
                atan2Deg(static_cast<double>(dir_y), static_cast<double>(dir_x));
            const double diff_deg = normalizeDeg(target_heading_deg - heading_deg);
            if (std::fabs(diff_deg) > kAngleEpsilonDeg) {
                pushChunkedRotate(plan.commands, diff_deg, max_rotate_deg);
                heading_deg = normalizeDeg(heading_deg + diff_deg);
            }
            const double run_distance_cm =
                std::sqrt(static_cast<double>(dir_x * dir_x + dir_y * dir_y)) * resolution_cm * run_length;
            pushChunkedAdvance(plan.commands, run_distance_cm, max_advance_cm);
        }

        prev = run_prev;
        i = run_end;
    }
    plan.final_heading_deg = heading_deg;
    return plan;
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

// Per-instance state persisted across nextStep() calls.
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

    // Deferred scan attached to the final command of the current movement queue.
    std::optional<Orientation> pending_moves_scan_orientation{};
    std::optional<VoxelIndex> pending_moves_scan_target{};
    std::optional<VoxelIndex> pending_moves_scan_frontier{};

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

    // Extends an in-lane Sweep batch across consecutive safe Empty cells.
    [[nodiscard]] std::vector<VoxelIndex> extendSweepBatch(const Context& ctx, const VoxelIndex& candidate,
                                                            long dx, long dy, long dz, const Position3D& offset,
                                                            const common_types::MapConfig& config);
    // Schedules a scan on the final command of a Sweep movement batch.
    void preparePipelinedSweepScan(const Context& ctx, const common_types::DroneState& state,
                                   const common_types::MapConfig& config, const VoxelIndex& batch_tail,
                                   const Position3D& offset, double final_heading_deg);
    // Schedules the Frontier scan on the final movement command.
    void preparePipelinedFrontierScan(const Context& ctx, const common_types::DroneState& state,
                                      const BfsResult& bfs, const Position3D& offset, double final_heading_deg);

    [[nodiscard]] std::optional<VoxelIndex> findUntriedTargetNear(const Context& ctx, const VoxelIndex& s);
    [[nodiscard]] BfsResult frontierBfs(const Context& ctx, const VoxelIndex& start, const Position3D& offset);
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
    common_types::MappingStepCommand result = movementOnlyCommand(cmd);

    // Attach a deferred scan only to the queue's final movement command.
    if (pending_moves.empty() && pending_moves_scan_orientation) {
        result.scan_orientation = pending_moves_scan_orientation;
        pending_scan_target = pending_moves_scan_target;
        pending_scan_frontier = pending_moves_scan_frontier;
        pending_scan_in_frontier_phase = pending_moves_scan_frontier.has_value();
        pending_moves_scan_orientation.reset();
        pending_moves_scan_target.reset();
        pending_moves_scan_frontier.reset();
    }
    return result;
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

    // Direct Sweep movement is allowed only to a safe face-adjacent candidate.
    const VoxelIndex candidate = sweep_candidate;
    const long dx = candidate.ix - cur.ix;
    const long dy = candidate.iy - cur.iy;
    const long dz = candidate.iz - cur.iz;
    const bool candidate_is_face_adjacent = isFaceAdjacent(candidate, cur);

    if (candidate_is_face_adjacent) {
        const common_types::VoxelOccupancy occ = ctx.map.atVoxel(toWorldCenter(candidate, config));

        if (isTargetOccupancy(occ) && sweep_repeat_scan_count < kMaxSweepScanAttempts) {
            return scanCommand(ctx, state, candidate, std::nullopt);
        }

        if (occ == common_types::VoxelOccupancy::Empty &&
            isSafeVoxel(ctx, candidate, intraVoxelOffset(ctx, state))) {
            const Position3D offset = intraVoxelOffset(ctx, state);

            const std::vector<VoxelIndex> batch = extendSweepBatch(ctx, candidate, dx, dy, dz, offset, config);
            const VoxelIndex batch_tail = batch.back();

            const MovementPlan plan = buildMovementQueue(ctx, state.heading, cur, batch);
            pending_moves = plan.commands;

            // Advance the Sweep cursor once for every batched cell.
            bool sweep_exhausted = false;
            for (std::size_t consumed = 0; consumed < batch.size(); ++consumed) {
                sweep_exhausted = advanceSweepCandidate();
            }
            if (sweep_exhausted) {
                phase = Phase::Frontier;
            }

            // Pipeline a scan for the next Sweep candidate onto the batch's final movement.
            if (!sweep_exhausted && !pending_moves.empty()) {
                preparePipelinedSweepScan(ctx, state, config, batch_tail, offset, plan.final_heading_deg);
            }

            if (!pending_moves.empty()) {
                return popPendingMove();
            }
            return frontierStep(ctx, state);
        }
        sweep_repeat_scan_count = 0;
    }

    // Let Frontier handle candidates that Sweep cannot enter directly.
    if (advanceSweepCandidate()) {
        phase = Phase::Frontier;
    }
    return frontierStep(ctx, state);
}

// Extends a pure in-lane Sweep batch across consecutive safe Empty cells.
std::vector<VoxelIndex> MappingAlgorithmImpl::Impl::extendSweepBatch(const Context& ctx,
                                                                     const VoxelIndex& candidate, long dx, long dy,
                                                                     long dz, const Position3D& offset,
                                                                     const common_types::MapConfig& config) {
    std::vector<VoxelIndex> batch{candidate};
    VoxelIndex batch_tail = candidate;
    if (dy == 0 && dz == 0 && dx == sweep_dir_x) {
        while (true) {
            const long peek_x = batch_tail.ix + sweep_dir_x;
            if (peek_x < sweep_min_x || peek_x > sweep_max_x) {
                break; // Stop before a lane pivot.
            }
            const VoxelIndex peek{peek_x, batch_tail.iy, batch_tail.iz};
            if (ctx.map.atVoxel(toWorldCenter(peek, config)) != common_types::VoxelOccupancy::Empty ||
                !isSafeVoxel(ctx, peek, offset)) {
                break;
            }
            batch.push_back(peek);
            batch_tail = peek;
        }
    }
    return batch;
}

// Attaches a scan for the next face-adjacent target to the batch's final movement.
void MappingAlgorithmImpl::Impl::preparePipelinedSweepScan(const Context& ctx, const common_types::DroneState& state,
                                                            const common_types::MapConfig& config,
                                                            const VoxelIndex& batch_tail, const Position3D& offset,
                                                            double final_heading_deg) {
    const VoxelIndex next_candidate = sweep_candidate;
    if (!isFaceAdjacent(next_candidate, batch_tail)) {
        return;
    }
    if (!isTargetOccupancy(ctx.map.atVoxel(toWorldCenter(next_candidate, config)))) {
        return;
    }
    const Position3D predicted_position = toWorldCenter(batch_tail, config) + offset;
    const Orientation predicted_heading{final_heading_deg * horizontal_angle[deg], state.heading.altitude};
    pending_moves_scan_orientation = relativeScanOrientation(
        predicted_position, toWorldCenter(next_candidate, config), predicted_heading);
    pending_moves_scan_target = next_candidate;
    pending_moves_scan_frontier.reset(); // Sweep-phase scan
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

BfsResult MappingAlgorithmImpl::Impl::frontierBfs(const Context& ctx, const VoxelIndex& start,
                                                    const Position3D& offset) {
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
            if (!isSafeVoxel(ctx, n, offset)) {
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
    // May be called as a temporary Sweep detour, so it does not force Phase::Frontier.

    const VoxelIndex cur = toVoxelIndex(state.position, ctx.map.getMapConfig());
    const Position3D offset = intraVoxelOffset(ctx, state);

    const BfsResult bfs = frontierBfs(ctx, cur, offset);
    if (!bfs.found) {
        // Final check distinguishes a complete map from unreachable unresolved voxels.
        final_status = hasAnyUnresolvedVoxel(ctx) ? common_types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                                   : common_types::AlgorithmStatus::Finished;
        phase = Phase::Done;
        return statusOnlyCommand(final_status);
    }

    if (bfs.frontier == cur) {
        return scanCommand(ctx, state, bfs.target, cur);
    }

    const MovementPlan plan = buildMovementQueue(ctx, state.heading, cur, bfs.path);
    pending_moves = plan.commands;
    if (pending_moves.empty()) {
        // Prevent an infinite stall if a non-trivial path yields no movement.
        final_status = common_types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        phase = Phase::Done;
        return statusOnlyCommand(final_status);
    }

    // Pipeline the Frontier scan onto the queue's final movement command.
    preparePipelinedFrontierScan(ctx, state, bfs, offset, plan.final_heading_deg);

    return popPendingMove();
}

// Predicts the post-movement state and schedules the Frontier scan.
void MappingAlgorithmImpl::Impl::preparePipelinedFrontierScan(const Context& ctx,
                                                               const common_types::DroneState& state,
                                                               const BfsResult& bfs, const Position3D& offset,
                                                               double final_heading_deg) {
    const Position3D predicted_position = toWorldCenter(bfs.frontier, ctx.map.getMapConfig()) + offset;
    const Orientation predicted_heading{final_heading_deg * horizontal_angle[deg], state.heading.altitude};
    const Position3D target_center = toWorldCenter(bfs.target, ctx.map.getMapConfig());
    pending_moves_scan_orientation = relativeScanOrientation(predicted_position, target_center, predicted_heading);
    pending_moves_scan_target = bfs.target;
    pending_moves_scan_frontier = bfs.frontier;
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

} // namespace algorithm_322889890_315113738
