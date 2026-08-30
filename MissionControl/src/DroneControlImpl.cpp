#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/AdvanceDirection.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mission_control_322889890_315113738 {

namespace common_types = common::types;

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// DroneStepStatus::Error is recorded but does not terminate the mission.
constexpr int kMaxBoundaryNudges = 64;

[[nodiscard]] double numCm(common::PhysicalLength v) { return v.force_numerical_value_in(cm); }

// Maximum retries for invalid or NOOP Algorithm commands.
constexpr int kMaxAlgorithmAttempts = 3;

// Maximum retries for invalid or NOOP Algorithm commands.
constexpr int kMaxLidarScanAttempts = 3;

// Maximum retries for an empty LiDAR scan.
constexpr int kMaxMovementAttempts = 3;

// Rejects non-finite values in active movement fields.
[[nodiscard]] bool isMovementCommandValid(const common_types::MovementCommand& movement) {
    switch (movement.type) {
        case common_types::MovementCommandType::Hover:
            return true;
        case common_types::MovementCommandType::Rotate:
            return std::isfinite(movement.angle.force_numerical_value_in(deg));
        case common_types::MovementCommandType::Advance:
        case common_types::MovementCommandType::Elevate:
            return std::isfinite(movement.distance.force_numerical_value_in(cm));
    }
    return true;
}

[[nodiscard]] bool isScanOrientationValid(const Orientation& orientation) {
    return std::isfinite(orientation.horizontal.force_numerical_value_in(deg)) &&
           std::isfinite(orientation.altitude.force_numerical_value_in(deg));
}

[[nodiscard]] bool isAlgorithmCommandValid(const common_types::MappingStepCommand& command) {
    if (command.movement.has_value() && !isMovementCommandValid(*command.movement)) {
        return false;
    }
    if (command.scan_orientation.has_value() && !isScanOrientationValid(*command.scan_orientation)) {
        return false;
    }
    return true;
}

// Row 4 ("The algorithm returned a NOOP"): only a Working status with neither movement nor scan
// is a faulty NOOP. Finished/FinishedWithUnmappableVoxels with no movement/scan are legitimate
// terminal results and must not be rejected here.
[[nodiscard]] bool isFaultyNoop(const common_types::MappingStepCommand& command) {
    return command.status == common_types::AlgorithmStatus::Working &&
           !command.movement.has_value() && !command.scan_orientation.has_value();
}

// Cartesian displacement produced by a position-changing movement.
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

// Tolerance for floating-point error when validating GPS against expected position.
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

// Estimates the largest displacement fraction that stays within one axis.
[[nodiscard]] double estimateLegalFraction(double start_cm, double delta_cm, double min_cm, double max_cm) {
    if (delta_cm > 0.0) {
        return (max_cm - start_cm) / delta_cm;
    }
    if (delta_cm < 0.0) {
        return (min_cm - start_cm) / delta_cm;
    }
    return 1.0;
}

// Finds the largest movement fraction accepted by output_map_.isInBounds().
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

// Safety factor for accumulated floating-point error while splitting movements.
constexpr double kSplitToleranceSafetyFactor = 4.0;

// Splits a signed magnitude into pieces no larger than max_magnitude.
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

// Splits a movement into chunks that respect the drone's configured limits.
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
                                   const common::ILidar& lidar,
                                   const common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
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

    // Retry invalid or NOOP Algorithm commands without advancing the step.
    common_types::MappingStepCommand command;
    bool accepted = false;
    for (int attempt = 0; attempt < kMaxAlgorithmAttempts; ++attempt) {
        command = mapping_algorithm_.nextStep(state, latest_scan_ptr);
        if (isAlgorithmCommandValid(command) && !isFaultyNoop(command)) {
            accepted = true;
            break;
        }
    }
    if (!accepted) {
        throw std::runtime_error(
            "DroneControlImpl::prepareNextSequence: the Algorithm returned an invalid or "
            "no-op command " + std::to_string(kMaxAlgorithmAttempts) + " times in a row");
    }

    PendingMovementSequence pending;
    pending.scan_orientation = command.scan_orientation;
    pending.status = command.status;
    pending.heading = state.heading;

    if (!command.movement.has_value()) {
        return pending;
    }

    common_types::MovementCommand movement = *command.movement;

    // Shorten position-changing movements to the largest legal in-bounds distance.
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

    // Shorten position-changing movements to the largest legal in-bounds distance.
    pending.movements = splitMovement(movement, drone_);
    return pending;
}

std::optional<common_types::DroneStepResult> DroneControlImpl::dispatchMovementAndValidateGps(
    const common_types::MovementCommand& movement, const Orientation& heading) {
    common_types::MovementResult result{};

    // Retry the same movement chunk when the driver reports failure.
    bool movement_succeeded = false;
    for (int attempt = 0; attempt < kMaxMovementAttempts; ++attempt) {
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

        if (result.success) {
            movement_succeeded = true;
            break;
        }
    }

    if (!movement_succeeded) {
        pending_sequence_.reset();
        throw std::runtime_error(
            "DroneControlImpl::step: the movement driver returned failure " +
            std::to_string(kMaxMovementAttempts) +
            " times in a row for the same movement chunk: " + result.message);
    }

    // Advance the expected position, then verify it against GPS.
    const Position3D expected_position = expectedPositionAfterMovement(*internal_position_, movement, heading);
    internal_position_ = expected_position;

    const std::optional<Position3D> validated_position = validatePostMovementGps(expected_position);
    if (!validated_position.has_value()) {
        // Discard the remaining sequence when the post-movement position cannot be verified.
        pending_sequence_.reset();
        return common_types::DroneStepResult{
            common_types::DroneStepStatus::Error,
            "DroneControlImpl::step: post-movement GPS position is inconsistent with the "
            "movement just executed, even after retrying"};
    }

    // A reading (the first, or a later retry) matched: resynchronize the internal estimate
    // to that accepted GPS value rather than leaving it at the merely-expected position.
    internal_position_ = *validated_position;
    return std::nullopt;
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

std::optional<common_types::DroneStepResult> DroneControlImpl::dispatchScanAndApplyToMap(
    const Orientation& scan_orientation, const Position3D& post_move_pos,
    const Orientation& post_move_heading) {
    // Retry empty scans; scan exceptions are returned immediately as errors.
    common_types::LidarScanResult scan;
    bool scan_accepted = false;
    for (int attempt = 0; attempt < kMaxLidarScanAttempts; ++attempt) {
        try {
            scan = lidar_.scan(scan_orientation);
        } catch (const std::exception& e) {
            return common_types::DroneStepResult{
                common_types::DroneStepStatus::Error, e.what()};
        }
        if (!scan.empty()) {
            scan_accepted = true;
            break;
        }
    }
    if (!scan_accepted) {
        throw std::runtime_error(
            "DroneControlImpl::step: the LiDAR returned an empty scan " +
            std::to_string(kMaxLidarScanAttempts) + " times in a row");
    }

    // Map-update failures become step errors without publishing the new scan.
    try {
        ScanResultToVoxels::applyToMap(
            output_map_, post_move_pos, post_move_heading, scan, lidar_.config());
    } catch (const std::exception& e) {
        return common_types::DroneStepResult{
            common_types::DroneStepStatus::Error, e.what()};
    }
    latest_scan_ = scan;
    return std::nullopt;
}

std::optional<common_types::DroneStepResult> DroneControlImpl::handlePreStepGps(
    Position3D& gps_position) {
    // Validates the GPS reading before starting the step.
    gps_position = gps_.position();

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
        // Ignore a bad GPS sample when the internal baseline remains valid.
        ++step_index_;
        return common_types::DroneStepResult{
            common_types::DroneStepStatus::Continue, "ignored out-of-bounds GPS reading"};
    }

    if (!internal_position_.has_value()) {
        internal_position_ = gps_position;
    }

    return std::nullopt;
}

common_types::DroneStepResult DroneControlImpl::step() {
    Position3D gps_position{};
    const std::optional<common_types::DroneStepResult> gps_error = handlePreStepGps(gps_position);
    if (gps_error.has_value()) {
        return *gps_error;
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

    // Movement precedes scanning; movement exceptions propagate unchanged.
    if (movement_to_dispatch) {
        const std::optional<common_types::DroneStepResult> movement_error =
            dispatchMovementAndValidateGps(*movement_to_dispatch, pending.heading);
        if (movement_error.has_value()) {
            return *movement_error;
        }
    }

    // Defer the command's scan and status until its final movement chunk.
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
        // Use the validated post-movement position as the scan origin when movement occurred.
        const Position3D post_move_pos = movement_to_dispatch ? *internal_position_ : gps_position;
        const Orientation post_move_heading = gps_.heading();

        const std::optional<common_types::DroneStepResult> scan_error =
            dispatchScanAndApplyToMap(*scan_orientation, post_move_pos, post_move_heading);
        if (scan_error.has_value()) {
            return *scan_error;
        }
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
                kUnmappableVoxelsMessage};
    }

    return common_types::DroneStepResult{
        common_types::DroneStepStatus::Error,
        "DroneControlImpl::step: unhandled AlgorithmStatus."};
}

common_types::DroneState DroneControlImpl::state() const {
    return common_types::DroneState{
        gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_322889890_315113738
