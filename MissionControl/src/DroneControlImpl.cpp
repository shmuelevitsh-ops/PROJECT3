#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/AdvanceDirection.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace MissionControl_322889890_315113738 {

namespace common_types = common::types;

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// Upper bound on how many times legalMovementFraction nudges its candidate fraction toward zero
// (via std::nextafter) while searching for one output_map_.isInBounds() actually accepts. Each
// nudge is the smallest possible step for a double, so this is far more headroom than any
// realistic candidate should need to converge.
constexpr int kMaxBoundaryNudges = 64;

[[nodiscard]] double numCm(common::PhysicalLength v) { return v.force_numerical_value_in(cm); }

// The (dx, dy, dz), in cm, that `movement` would apply from `state` if run at full requested
// distance. Only Advance (x/y, via current heading) and Elevate (z) change position; the caller
// only calls this for those two types. Advance's direction uses the shared
// user_common_322889890_315113738::advanceDirection() -- the same one MockMovement::advance()
// itself uses to actually move the drone -- so this prediction and the real post-movement GPS
// position it is later checked against (Optional Common-Issues row 12) can never disagree merely
// from the two sides computing cos/sin direction differently.
struct MovementDelta {
    double dx_cm = 0.0;
    double dy_cm = 0.0;
    double dz_cm = 0.0;
};

[[nodiscard]] MovementDelta movementDelta(const common_types::MovementCommand& movement,
                                           const common_types::DroneState& state) {
    const double distance_cm = numCm(movement.distance);

    if (movement.type == common_types::MovementCommandType::Advance) {
        const user_common_322889890_315113738::AdvanceDirection direction =
            user_common_322889890_315113738::advanceDirection(state.heading.horizontal);
        return MovementDelta{direction.x * distance_cm, direction.y * distance_cm, 0.0};
    }

    // Elevate: signed distance moves z up (positive) or down (negative).
    return MovementDelta{0.0, 0.0, distance_cm};
}

[[nodiscard]] Position3D applyDelta(const Position3D& start, const MovementDelta& delta, double t) {
    return Position3D{
        start.x + t * delta.dx_cm * x_extent[cm],
        start.y + t * delta.dy_cm * y_extent[cm],
        start.z + t * delta.dz_cm * z_extent[cm],
    };
}

// Where the internal position estimate should land after `movement` (the already-amended/split
// chunk actually dispatched, not the Algorithm's original request) executes from `start` under
// `heading` (Optional Common-Issues row 12). Rotate/Hover never change position; Advance/Elevate
// reuse the same delta math as the OOB-amendment path above.
[[nodiscard]] Position3D expectedPositionAfterMovement(const Position3D& start,
                                                        const common_types::MovementCommand& movement,
                                                        const Orientation& heading) {
    if (movement.type != common_types::MovementCommandType::Advance &&
        movement.type != common_types::MovementCommandType::Elevate) {
        return start;
    }
    const common_types::DroneState pseudo_state{start, heading, 0};
    const MovementDelta delta = movementDelta(movement, pseudo_state);
    return applyDelta(start, delta, 1.0);
}

// Numerical (not physical) tolerance for comparing a post-movement GPS reading against the
// physically-expected internal position (Optional Common-Issues row 12). Sized to absorb only
// ordinary double/trigonometric rounding -- e.g. si::cos/sin of a heading, scaled by a large
// dispatched distance -- never a genuine GPS discrepancy. A fixed fraction of magnitude (e.g.
// 1e-9 * scale) scales the wrong way: at a coordinate of 1e9cm it would already tolerate ~1cm of
// real error, which is a physical "close enough" range, not a rounding artifact. Instead the
// relative term is machine-epsilon-based -- each floating-point operation in the expected-position
// computation (a trig call, a multiplication, an addition) can introduce at most about one ULP of
// relative error at the operands' own magnitude, so a small constant multiple of
// std::numeric_limits<double>::epsilon() times that magnitude safely bounds the accumulated
// rounding without ever approaching a genuine physical discrepancy. kGpsPositionAbsoluteFloorCm
// only matters near zero, where the epsilon-scaled term itself vanishes.
constexpr double kGpsPositionAbsoluteFloorCm = 1e-9;
constexpr double kGpsPositionEpsilonSafetyFactor = 8.0;

[[nodiscard]] bool nearlyEqualCm(double a_cm, double b_cm) {
    const double diff = std::abs(a_cm - b_cm);
    const double scale = std::max(std::abs(a_cm), std::abs(b_cm));
    const double tolerance = kGpsPositionAbsoluteFloorCm +
                              kGpsPositionEpsilonSafetyFactor * std::numeric_limits<double>::epsilon() * scale;
    return diff <= tolerance;
}

[[nodiscard]] bool positionsMatch(const Position3D& a, const Position3D& b) {
    return nearlyEqualCm(a.x.force_numerical_value_in(cm), b.x.force_numerical_value_in(cm)) &&
           nearlyEqualCm(a.y.force_numerical_value_in(cm), b.y.force_numerical_value_in(cm)) &&
           nearlyEqualCm(a.z.force_numerical_value_in(cm), b.z.force_numerical_value_in(cm));
}

// Number of extra gps_.position() reads attempted after an initial post-movement reading that
// disagrees with the physically-expected position, before giving up (Optional Common-Issues row
// 12).
constexpr int kGpsPostMovementRetryAttempts = 3;

// Largest fraction t in [0,1] of a displacement `delta_cm` (starting at `start_cm`) that, by
// getMapConfig().boundaries' numbers alone, would stay within [min_cm, max_cm]. This is only a
// first estimate for legalMovementFraction below -- it deliberately does not assume which side of
// [min_cm, max_cm] is inclusive; output_map_.isInBounds() is what actually decides that. A zero
// delta is unconstrained along this axis.
[[nodiscard]] double estimateLegalFraction(double start_cm, double delta_cm, double min_cm, double max_cm) {
    if (delta_cm > 0.0) {
        return (max_cm - start_cm) / delta_cm;
    }
    if (delta_cm < 0.0) {
        return (min_cm - start_cm) / delta_cm;
    }
    return 1.0;
}

// Fraction (in [0,1]) of a position-changing movement's full requested distance that is legal
// from `state`, according to `output_map`: output_map.isInBounds() is the sole authority on
// legality here. getMapConfig().boundaries is used only to produce a first estimate of how much
// distance should fit; that estimate is then verified against isInBounds(), and nudged toward the
// start position with std::nextafter (never by a fixed physical distance) if the map's own
// in-bounds semantics turn out to exclude it. Returns 0.0 if no non-zero legal fraction is found
// within kMaxBoundaryNudges nudges.
[[nodiscard]] double legalMovementFraction(const common::IMutableMap3D& output_map,
                                            const MovementDelta& delta,
                                            const common_types::DroneState& state,
                                            const common_types::MappingBounds& bounds) {
    double t = std::clamp(
        std::min({1.0,
                  estimateLegalFraction(state.position.x.force_numerical_value_in(cm), delta.dx_cm,
                                        bounds.min_x.force_numerical_value_in(cm),
                                        bounds.max_x.force_numerical_value_in(cm)),
                  estimateLegalFraction(state.position.y.force_numerical_value_in(cm), delta.dy_cm,
                                        bounds.min_y.force_numerical_value_in(cm),
                                        bounds.max_y.force_numerical_value_in(cm)),
                  estimateLegalFraction(state.position.z.force_numerical_value_in(cm), delta.dz_cm,
                                        bounds.min_height.force_numerical_value_in(cm),
                                        bounds.max_height.force_numerical_value_in(cm))}),
        0.0, 1.0);

    for (int attempt = 0; attempt <= kMaxBoundaryNudges && t > 0.0; ++attempt) {
        if (output_map.isInBounds(applyDelta(state.position, delta, t))) {
            return t;
        }
        t = std::nextafter(t, 0.0);
    }
    return 0.0;
}

// Safety-factor multiplier applied to the true double-rounding-error bound used to decide
// whether a post-subtraction residue in splitMagnitude() is genuine floating-point noise. A
// fixed fraction *of max_magnitude* (e.g. 1e-9 * max_magnitude) scales the wrong way: with a huge
// max_magnitude (e.g. 1e9cm) it would silently swallow a real, much smaller remainder (e.g.
// 0.5cm). The bound used instead is derived from std::numeric_limits<double>::epsilon() and the
// scale of `magnitude` itself -- each of the N subtractions performed can introduce at most about
// one ULP of rounding error at that scale, so (N+1) * epsilon * magnitude is a safe upper bound on
// the total accumulated error; this factor just pads that bound a little further.
constexpr double kSplitToleranceSafetyFactor = 4.0;

// Splits `total` (a signed magnitude, e.g. cm or deg) into pieces of magnitude at most
// `max_magnitude`, each carrying the same sign as `total`, summing to `total` within ordinary
// floating-point tolerance. Returns a single-element {total} -- i.e. "no split needed" -- when
// `max_magnitude` is non-positive (an unconfigured/degenerate limit never splits) or `total`
// already fits within one piece; the caller relies on that single-element result to mean
// "dispatch unchanged" (checking pieces.size() <= 1), so this never returns a single-element
// vector for any other reason.
[[nodiscard]] std::vector<double> splitMagnitude(double total, double max_magnitude) {
    const double magnitude = std::abs(total);
    if (!(max_magnitude > 0.0) || magnitude <= max_magnitude) {
        return {total};
    }

    const double sign = total < 0.0 ? -1.0 : 1.0;
    std::vector<double> pieces;
    double remaining = magnitude;
    int num_subtractions = 0;
    while (remaining > max_magnitude) {
        pieces.push_back(sign * max_magnitude);
        remaining -= max_magnitude;
        ++num_subtractions;
    }

    // Discards only genuine floating-point residue (e.g. ~1.4e-16cm left after ten 0.1cm
    // subtractions from 1.0cm) -- never a real remainder, however small relative to
    // max_magnitude (e.g. 0.5cm left over against a 1e9cm max).
    const double tolerance = kSplitToleranceSafetyFactor * static_cast<double>(num_subtractions + 1) *
                              std::numeric_limits<double>::epsilon() * magnitude;
    if (remaining > tolerance) {
        // A genuine final chunk. Clamped defensively so it can never exceed max_magnitude, in
        // case floating-point rounding in the repeated subtraction above ever left `remaining` a
        // hair above it instead of below.
        pieces.push_back(sign * std::min(remaining, max_magnitude));
    }
    return pieces;
}

// Splits a single movement into DroneConfigData-sized chunks (Optional Common-Issues: "Algorithm
// returned a movement bigger than the max allowed" -- a defense against a faulty Algorithm; a
// valid one already respects these limits). `movement` is assumed to already be the *legal*
// movement (post OOB-amendment for Advance/Elevate): this function only ever shortens for size,
// never re-checks map bounds. Hover is never split. A movement that already fits within its
// type's limit is returned as a single unchanged chunk.
[[nodiscard]] std::deque<common_types::MovementCommand> splitMovement(
    const common_types::MovementCommand& movement, const common_types::DroneConfigData& drone) {
    std::deque<common_types::MovementCommand> chunks;

    if (movement.type == common_types::MovementCommandType::Hover) {
        chunks.push_back(movement);
        return chunks;
    }

    if (movement.type == common_types::MovementCommandType::Rotate) {
        const double angle_deg = movement.angle.force_numerical_value_in(deg);
        const std::vector<double> pieces =
            splitMagnitude(angle_deg, drone.max_rotate.force_numerical_value_in(deg));
        if (pieces.size() <= 1) {
            chunks.push_back(movement);
            return chunks;
        }
        for (const double piece_deg : pieces) {
            common_types::MovementCommand chunk = movement;
            chunk.angle = movement.angle * (piece_deg / angle_deg);
            chunks.push_back(chunk);
        }
        return chunks;
    }

    // Advance / Elevate: split the signed distance, preserving direction in every chunk.
    const double distance_cm = numCm(movement.distance);
    const double max_cm = numCm(movement.type == common_types::MovementCommandType::Advance
                                     ? drone.max_advance
                                     : drone.max_elevate);
    const std::vector<double> pieces = splitMagnitude(distance_cm, max_cm);
    if (pieces.size() <= 1) {
        chunks.push_back(movement);
        return chunks;
    }
    for (const double piece_cm : pieces) {
        common_types::MovementCommand chunk = movement;
        chunk.distance = movement.distance * (piece_cm / distance_cm);
        chunks.push_back(chunk);
    }
    return chunks;
}

} // namespace

DroneControlImpl::DroneControlImpl(common_types::DroneConfigData drone,
                                   common_types::MissionConfigData mission,
                                   common::ILidar& lidar,
                                   common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

DroneControlImpl::PendingMovementSequence DroneControlImpl::prepareNextSequence(
    const Position3D& gps_position) {
    const common_types::DroneState state{gps_position, gps_.heading(), step_index_};
    const common_types::LidarScanResult* latest_scan_ptr =
        latest_scan_ ? &(*latest_scan_) : nullptr;
    const common_types::MappingStepCommand command =
        mapping_algorithm_.nextStep(state, latest_scan_ptr);

    PendingMovementSequence pending;
    pending.scan_orientation = command.scan_orientation;
    pending.status = command.status;
    pending.heading = state.heading;

    if (!command.movement.has_value()) {
        return pending;
    }

    common_types::MovementCommand movement = *command.movement;

    // Out-of-bounds handling (Optional Common-Issues scenario) applies first, to establish the
    // total *legal* movement -- Advance/Elevate are the only position-changing movement types.
    // output_map_.isInBounds() is the sole authority on whether a destination is legal: a
    // movement whose full destination it accepts is legal unchanged; one it rejects is shortened
    // -- using getMapConfig().boundaries only to estimate how much distance should fit, then
    // re-verified against isInBounds() itself (see legalMovementFraction) -- to the maximum legal
    // non-zero distance in the same direction. If no such distance remains (the drone is already
    // at the relevant boundary), no movement chunk is produced at all, and the rest of the step
    // (scan, status, step_index_) proceeds normally.
    if (movement.type == common_types::MovementCommandType::Advance ||
        movement.type == common_types::MovementCommandType::Elevate) {
        const MovementDelta delta = movementDelta(movement, state);

        if (!output_map_.isInBounds(applyDelta(state.position, delta, 1.0))) {
            const double fraction =
                legalMovementFraction(output_map_, delta, state, output_map_.getMapConfig().boundaries);

            if (fraction <= 0.0) {
                return pending; // no legal movement -- pending.movements stays empty.
            }
            movement.distance = movement.distance * fraction;
        }
    }

    // Oversized-command handling (Optional Common-Issues scenario): once the legal movement is
    // known, split it into DroneConfigData-sized chunks -- a defense against a faulty Algorithm
    // that requested more than drone_'s configured max in one command.
    pending.movements = splitMovement(movement, drone_);
    return pending;
}

std::optional<Position3D> DroneControlImpl::validatePostMovementGps(const Position3D& expected) {
    // A reading is only accepted if it is both numerically consistent with `expected` *and* in
    // bounds. Row 10 already guarantees the dispatched position-changing movement's destination
    // is legal, so a reading that is out of bounds cannot be the real post-movement position no
    // matter how numerically close it is to `expected` -- it must still be treated as a row-12
    // mismatch and retried like any other.
    const Position3D first_reading = gps_.position();
    if (output_map_.isInBounds(first_reading) && positionsMatch(first_reading, expected)) {
        return first_reading;
    }
    for (int attempt = 0; attempt < kGpsPostMovementRetryAttempts; ++attempt) {
        const Position3D retry_reading = gps_.position();
        if (output_map_.isInBounds(retry_reading) && positionsMatch(retry_reading, expected)) {
            return retry_reading;
        }
    }
    return std::nullopt;
}

common_types::DroneStepResult DroneControlImpl::step() {
    // Optional Common-Issues row 11 ("The GPS returns out-of-bound coordinates"): read GPS once,
    // before consulting the Algorithm or dispatching any pending chunk, and use output_map_ (the
    // sole authority on legality elsewhere in this class) to judge it.
    const Position3D gps_position = gps_.position();

    if (!output_map_.isInBounds(gps_position)) {
        if (!internal_position_.has_value()) {
            // No internal baseline exists yet to judge this reading against -- nothing to fall
            // back on.
            throw std::runtime_error(
                "DroneControlImpl::step: GPS returned an out-of-bounds position and no internal "
                "position baseline exists yet to validate it against");
        }
        if (!output_map_.isInBounds(*internal_position_)) {
            // The one reference we have is itself out of bounds -- no basis left to call this
            // GPS sample spurious rather than real.
            throw std::runtime_error(
                "DroneControlImpl::step: GPS returned an out-of-bounds position and the internal "
                "position baseline is also out of bounds");
        }
        // A plausible internal baseline says this GPS sample is spurious: ignore it entirely --
        // no Algorithm call, no movement, no scan/map write, and never substitute
        // internal_position_ in place of the rejected reading. Any pending oversized-movement
        // sequence (row 8) is left untouched, since no chunk was dispatched this step, so a later
        // valid step resumes the same chunk without recalling the Algorithm. This still counts as
        // a step so that repeated bad samples consume the max_steps budget instead of looping
        // forever.
        ++step_index_;
        return common_types::DroneStepResult{
            common_types::DroneStepStatus::Continue, "ignored out-of-bounds GPS reading"};
    }

    if (!internal_position_.has_value()) {
        internal_position_ = gps_position;
    }

    if (!pending_sequence_) {
        // Only fetched once per Algorithm-issued command: while a prior step() left chunks (and/or
        // a deferred scan/status) pending, mapping_algorithm_.nextStep() must not be called again
        // until they are all dispatched.
        pending_sequence_ = prepareNextSequence(gps_position);
    }
    PendingMovementSequence& pending = *pending_sequence_;

    std::optional<common_types::MovementCommand> movement_to_dispatch;
    if (!pending.movements.empty()) {
        movement_to_dispatch = pending.movements.front();
        pending.movements.pop_front();
    }

    // Movement is executed before any scan, per MappingStepCommand's contract. Movement is the
    // one dispatch step allowed to throw (e.g. MockMovement rejecting a real wall collision it
    // alone can see): caught narrowly here, without touching the scan block below, so a
    // collision is reported the same way as a movement that returns success=false, and never
    // reaches lidar_.scan() or increments step_index_. Any remaining chunks of this sequence are
    // discarded: the whole original command is being reported as failed, so there is no partial
    // sequence left to resume.
    if (movement_to_dispatch) {
        const common_types::MovementCommand& movement = *movement_to_dispatch;
        common_types::MovementResult result{};

        try {
            switch (movement.type) {
                case common_types::MovementCommandType::Hover:
                    break;
                case common_types::MovementCommandType::Rotate:
                    result = movement_.rotate(movement.rotation, movement.angle);
                    break;
                case common_types::MovementCommandType::Advance:
                    result = movement_.advance(movement.distance);
                    break;
                case common_types::MovementCommandType::Elevate:
                    result = movement_.elevate(movement.distance);
                    break;
            }
        } catch (const std::exception& e) {
            pending_sequence_.reset();
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Error, e.what()};
        }

        if (!result.success) {
            pending_sequence_.reset();
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Error, result.message};
        }

        // Optional Common-Issues row 12 ("A movement executed but GPS updated with impossible
        // coordinates"): the movement physically succeeded, so the internal estimate advances to
        // where it should now be -- computed from the chunk actually dispatched above, under the
        // heading this whole sequence was legalized/split under -- regardless of what GPS reports
        // next.
        const Position3D expected_position =
            expectedPositionAfterMovement(*internal_position_, movement, pending.heading);
        internal_position_ = expected_position;

        const std::optional<Position3D> validated_position = validatePostMovementGps(expected_position);
        if (!validated_position.has_value()) {
            // Every reading -- the first plus kGpsPostMovementRetryAttempts retries -- disagrees
            // with the physically-expected position: the drone's true position is now
            // unverified, so no scan is taken and any remaining chunks of this oversized sequence
            // are discarded rather than dispatched against an unverified location.
            // internal_position_ stays at expected_position (set above), and, per the existing
            // Error-path convention, step_index_ is not incremented for this failing step.
            pending_sequence_.reset();
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Error,
                "DroneControlImpl::step: post-movement GPS position is inconsistent with the "
                "movement just executed, even after retrying"};
        }

        // A reading (the first, or a later retry) matched: resynchronize the internal estimate
        // to that accepted GPS value rather than leaving it at the merely-expected position.
        internal_position_ = *validated_position;
    }

    // A chunk before the last one in a split sequence: this step() dispatched real movement (one
    // chunk = one step, incrementing step_index_ exactly once), but the original command's scan
    // and status are deferred to the final chunk, so MissionControl cannot see Finished/
    // FinishedWithUnmappableVoxels -- and stop the mission -- before the whole legal movement has
    // actually executed.
    if (!pending.movements.empty()) {
        ++step_index_;
        latest_scan_ = std::nullopt;
        return common_types::DroneStepResult{
            common_types::DroneStepStatus::Continue, "working"};
    }

    const std::optional<Orientation> scan_orientation = pending.scan_orientation;
    const common_types::AlgorithmStatus status = pending.status;
    pending_sequence_.reset();

    if (scan_orientation.has_value()) {
        // Movement + scan: `internal_position_` was just resynchronized above to the validated
        // post-movement GPS reading (row 12), so reuse it as the scan origin rather than reading
        // GPS a second time. No movement / scan-only: reuse the pre-step GPS reading that already
        // passed row-11 validation -- never substitute internal_position_ for a GPS sample that
        // was itself ignored/rejected.
        const Position3D post_move_pos = movement_to_dispatch ? *internal_position_ : gps_position;
        const Orientation post_move_heading = gps_.heading();
        const common_types::LidarScanResult scan =
            lidar_.scan(*scan_orientation);

        ScanResultToVoxels::applyToMap(
            output_map_, post_move_pos, post_move_heading, scan, lidar_.config());
        latest_scan_ = scan;
    } else {
        latest_scan_ = std::nullopt;
    }

    ++step_index_;

    switch (status) {
        case common_types::AlgorithmStatus::Working:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Continue, "working"};

        case common_types::AlgorithmStatus::Finished:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Completed, "mapping finished"};

        case common_types::AlgorithmStatus::FinishedWithUnmappableVoxels:
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Completed,
                "mapping finished with unmappable voxels remaining"};
    }

    return common_types::DroneStepResult{
        common_types::DroneStepStatus::Error,
        "DroneControlImpl::step: unhandled AlgorithmStatus."};
}

common_types::DroneState DroneControlImpl::state() const {
    return common_types::DroneState{
        gps_.position(), gps_.heading(), step_index_};
}

} // namespace MissionControl_322889890_315113738
