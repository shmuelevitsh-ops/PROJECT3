#include <Simulator/Map3DImpl.h>

#include <Simulator/SimulationException.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <typeinfo>
#include <utility>

namespace simulator {

namespace types = common::types;
namespace isq = common::isq;

using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

// Raw values stored in the backing NpyArray.
constexpr int kRawPotentiallyOccupied = -3;
constexpr int kRawEmpty = 0;
constexpr int kRawOccupied = 1;
constexpr int kRawUnmapped = -1;

using VoxelIndex = std::array<long, 3>;

// Converts a world position into voxel indices using the formula:
//   idx = floor((world_coordinate - offset) / resolution)
// Returns std::nullopt if the map has no usable resolution (e.g. a default
// MapConfig), since indices cannot be derived in that case.
[[nodiscard]] std::optional<VoxelIndex> computeIndex(const Position3D& pos,
                                                       const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (!(resolution_cm > 0.0)) {
        return std::nullopt;
    }

    const double rel_x = (pos.x - config.offset.x).force_numerical_value_in(cm);
    const double rel_y = (pos.y - config.offset.y).force_numerical_value_in(cm);
    const double rel_z = (pos.z - config.offset.z).force_numerical_value_in(cm);

    return VoxelIndex{
        static_cast<long>(std::floor(rel_x / resolution_cm)),
        static_cast<long>(std::floor(rel_y / resolution_cm)),
        static_cast<long>(std::floor(rel_z / resolution_cm)),
    };
}

// True iff `pos` lies within `bounds` on every axis, using half-open
// intervals (min <= coordinate < max). This is the configured-boundaries
// check used by isInBounds()/atVoxel()/set() — distinct from, and checked
// before, the defensive NpyArray-index validity check in flatIndex().
[[nodiscard]] bool isWithinBoundaries(const Position3D& pos, const types::MappingBounds& bounds) {
    return pos.x >= bounds.min_x && pos.x < bounds.max_x &&
           pos.y >= bounds.min_y && pos.y < bounds.max_y &&
           pos.z >= bounds.min_height && pos.z < bounds.max_height;
}

// Converts voxel indices into a flat, row-major offset into the NpyArray data
// (matching the [X, Y, Z] shape used by the map files). Returns std::nullopt
// if the index falls outside the array - the caller must treat this as
// OutOfBounds *before* touching the array data.
[[nodiscard]] std::optional<std::size_t> flatIndex(const VoxelIndex& index,
                                                     const NpyArray::shape_t& shape) {
    if (shape.size() != 3) {
        return std::nullopt;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (index[axis] < 0 || static_cast<std::size_t>(index[axis]) >= shape[axis]) {
            return std::nullopt;
        }
    }
    const auto x = static_cast<std::size_t>(index[0]);
    const auto y = static_cast<std::size_t>(index[1]);
    const auto z = static_cast<std::size_t>(index[2]);
    return x * shape[1] * shape[2] + y * shape[2] + z;
}

// Reads the raw integer stored at a flat index. Supports signed int (our own
// output maps, so Unmapped can be represented as -1) plus the dtypes a hidden
// map loaded from disk may legitimately arrive in per the assignment's
// forum clarification (values are guaranteed to be only 0/1 -- Empty/
// Occupied -- for these): uint8, int8, and 4-byte unsigned int.
[[nodiscard]] long readRaw(const NpyArray& array, std::size_t flat) {
    if (array.ValueType() == typeid(int)) {
        return static_cast<long>(array.Data<int>()[flat]);
    }
    if (array.ValueType() == typeid(std::uint8_t)) {
        return static_cast<long>(array.Data<std::uint8_t>()[flat]);
    }
    if (array.ValueType() == typeid(char)) {
        return static_cast<long>(array.Data<char>()[flat]);
    }
    if (array.ValueType() == typeid(unsigned int)) {
        return static_cast<long>(array.Data<unsigned int>()[flat]);
    }
    std::cerr << "Map3DImpl: unsupported NpyArray value type; treating voxel as Unmapped.\n";
    return kRawUnmapped;
}

// Maps a raw stored integer to the corresponding VoxelOccupancy.
// Note: -2 is the OutOfBounds sentinel returned by atVoxel() itself and
// should never appear as a stored value. If it (or any other unexpected
// value) does appear in the array, it must not be silently treated as
// Unmapped without logging.
[[nodiscard]] types::VoxelOccupancy rawToOccupancy(long raw) {
    switch (raw) {
        case kRawPotentiallyOccupied:
            return types::VoxelOccupancy::PotentiallyOccupied;
        case kRawEmpty:
            return types::VoxelOccupancy::Empty;
        case kRawOccupied:
            return types::VoxelOccupancy::Occupied;
        case kRawUnmapped:
            return types::VoxelOccupancy::Unmapped;
        default:
            std::cerr << "Map3DImpl: unexpected stored voxel value " << raw
                      << "; treating as Unmapped.\n";
            return types::VoxelOccupancy::Unmapped;
    }
}

// Writes a VoxelOccupancy value to the backing array, honoring the dtype of
// the array (see readRaw for why two dtypes are supported).
void writeRaw(NpyArray& array, std::size_t flat, types::VoxelOccupancy value) {
    int raw = kRawUnmapped;
    switch (value) {
        case types::VoxelOccupancy::PotentiallyOccupied:
            raw = kRawPotentiallyOccupied;
            break;
        case types::VoxelOccupancy::Empty:
            raw = kRawEmpty;
            break;
        case types::VoxelOccupancy::Occupied:
            raw = kRawOccupied;
            break;
        case types::VoxelOccupancy::Unmapped:
            raw = kRawUnmapped;
            break;
        case types::VoxelOccupancy::OutOfBounds:
            // OutOfBounds is a read-only sentinel, not a stored voxel value.
            return;
    }

    if (array.ValueType() == typeid(int)) {
        array.Data<int>()[flat] = raw;
        return;
    }
    if (array.ValueType() == typeid(std::uint8_t)) {
        if (raw < 0) {
            std::cerr << "Map3DImpl: cannot store negative voxel value in a uint8 map; "
                         "ignoring set().\n";
            return;
        }
        array.Data<std::uint8_t>()[flat] = static_cast<std::uint8_t>(raw);
        return;
    }
    throw std::invalid_argument("Map3DImpl: unsupported NpyArray value type.");
}

// Number of voxels spanned by [0, span_cm) at the given resolution.
[[nodiscard]] std::size_t axisVoxelCount(double span_cm, double resolution_cm) {
    if (span_cm <= 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(std::ceil(span_cm / resolution_cm));
}

// Allocates a fresh, owned NpyArray sized from config.boundaries,
// config.offset, and config.resolution, initialized to Unmapped. Used for
// maps constructed with an empty NpyArray (e.g. a new output map).
//
// Sized from offset, not from the boundary minimum: array index 0
// corresponds to world position `offset` (per computeIndex()'s
// floor((pos - offset) / resolution) formula), so the array must span
// [offset, boundaries.max) on every axis — not just [boundaries.min,
// boundaries.max) — for every position inside the configured boundaries to
// be indexable without being clamped away by clampBoundariesToExtent()
// afterward. (rejectBoundariesStartingBeforeOffset() already guarantees
// offset <= boundaries.min, so this never shrinks the configured span.)
void allocateFreshMap(std::shared_ptr<NpyArray>& map_ptr, const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);

    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    if (resolution_cm > 0.0) {
        const auto& bounds = config.boundaries;
        const auto& offset = config.offset;
        nx = axisVoxelCount((bounds.max_x - offset.x).force_numerical_value_in(cm), resolution_cm);
        ny = axisVoxelCount((bounds.max_y - offset.y).force_numerical_value_in(cm), resolution_cm);
        nz = axisVoxelCount((bounds.max_height - offset.z).force_numerical_value_in(cm),
                             resolution_cm);
    }

    map_ptr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(int),
                                          NpyArray::GetTypeChar(typeid(int)));
    map_ptr->Allocate();
    std::fill_n(map_ptr->Data<int>(), map_ptr->NumValue(), kRawUnmapped);
}

// Validates an NpyArray loaded from disk (e.g. a hidden map). Throws
// std::invalid_argument if it cannot be interpreted as a [X, Y, Z] occupancy
// grid.
void validateLoadedArray(const NpyArray& array) {
    if (array.Shape().size() != 3) {
        throw SimulationException("INVALID_MAP_FORMAT", "Map3DImpl requires a 3D NpyArray with shape [X, Y, Z].");
    }
    if (array.ColMajor()) {
        throw SimulationException("INVALID_MAP_FORMAT", "Map3DImpl requires a row-major NpyArray.");
    }
    if (array.ValueType() != typeid(int) && array.ValueType() != typeid(std::uint8_t) &&
        array.ValueType() != typeid(char) && array.ValueType() != typeid(unsigned int)) {
        throw SimulationException("INVALID_MAP_FORMAT",
                                   "Map3DImpl requires an int, uint8, int8, or unsigned-int NpyArray.");
    }
}

// Resolution used by the single-argument constructor's inferred default
// MapConfig (see inferDefaultConfig()).
constexpr double kDefaultResolutionCm = 1.0;

// Infers a usable default MapConfig (resolution=1cm, offset=0, boundaries
// matching the array shape at 1cm resolution) from a non-empty, valid 3D
// NpyArray. Used by the single-argument constructor when no MapConfig is
// supplied. Throws std::invalid_argument if map_ptr is null or the array is
// empty, since no shape is available to infer geometry from in either case.
[[nodiscard]] types::MapConfig inferDefaultConfig(const std::shared_ptr<NpyArray>& map_ptr) {
    if (!map_ptr) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    if (map_ptr->IsEmpty()) {
        throw std::invalid_argument(
            "Map3DImpl: cannot infer default geometry from an empty NpyArray; "
            "construct with an explicit MapConfig instead.");
    }
    validateLoadedArray(*map_ptr);

    const NpyArray::shape_t& shape = map_ptr->Shape();
    const PhysicalLength resolution = kDefaultResolutionCm * isq::length[cm];

    std::cerr << "Map3DImpl: no MapConfig provided; inferred default geometry from the "
                 "NpyArray shape (resolution=1cm, offset=0). This is a temporary fallback — "
                 "callers should supply an explicit MapConfig.\n";

    return types::MapConfig{
        types::MappingBounds{
            0.0 * x_extent[cm], static_cast<double>(shape[0]) * x_extent[cm],
            0.0 * y_extent[cm], static_cast<double>(shape[1]) * y_extent[cm],
            0.0 * z_extent[cm], static_cast<double>(shape[2]) * z_extent[cm]},
        Position3D{},
        resolution};
}

// Throws std::invalid_argument if config.boundaries has min >= max on any
// axis. This is the *pre-clamp* invalid-configuration case: a genuinely
// malformed config, distinct from the post-clamp disjoint-boundaries case
// in rejectDisjointBoundaries() (where valid boundaries simply don't
// overlap the array's physical extent).
void rejectInvalidBoundaries(const types::MappingBounds& bounds) {
    if (bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y &&
        bounds.min_height < bounds.max_height) {
        return;
    }
    throw SimulationException("MISSION_BOUNDARY_INVALID",
                              "Map3DImpl: invalid mapping boundaries: min must be smaller than max.");
}

// Throws std::invalid_argument if any configured boundary minimum maps to a
// negative index under the configured offset and resolution, i.e. the
// configured boundaries start before the map's offset on some axis. Uses
// the same index formula as computeIndex(): floor((min - offset) / resolution).
// Only relevant for fresh-map allocation from an empty NpyArray — config_
// here is otherwise known to have a positive resolution by this point.
void rejectBoundariesStartingBeforeOffset(const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);

    const double min_x_index =
        std::floor((config.boundaries.min_x - config.offset.x).force_numerical_value_in(cm) /
                    resolution_cm);
    const double min_y_index =
        std::floor((config.boundaries.min_y - config.offset.y).force_numerical_value_in(cm) /
                    resolution_cm);
    const double min_z_index =
        std::floor((config.boundaries.min_height - config.offset.z).force_numerical_value_in(cm) /
                    resolution_cm);

    if (min_x_index >= 0.0 && min_y_index >= 0.0 && min_z_index >= 0.0) {
        return;
    }
    throw SimulationException("MISSION_BOUNDARY_INVALID",
                              "Map3DImpl: configured boundaries start before the map offset.");
}

// Clamps a single axis of config.boundaries to [extent_min_cm, extent_max_cm]
// (the physical extent of the backing array on that axis) and logs if any
// clamping occurred, per the Boundary Validation Policy.
void clampAxis(double& min_cm, double& max_cm, double extent_min_cm, double extent_max_cm,
               const char* axis_name) {
    const double original_min = min_cm;
    const double original_max = max_cm;

    if (min_cm < extent_min_cm) {
        min_cm = extent_min_cm;
    }
    if (max_cm > extent_max_cm) {
        max_cm = extent_max_cm;
    }

    if (min_cm != original_min || max_cm != original_max) {
        std::cerr << "Map3DImpl: clamped " << axis_name << " mapping boundary from ["
                  << original_min << ", " << original_max << "] cm to [" << min_cm << ", "
                  << max_cm << "] cm to fit the map extent.\n";
    }
}

// Clamps config.boundaries so that every position within them maps to a
// valid index into `shape`. No-op if the map has no usable resolution.
void clampBoundariesToExtent(types::MapConfig& config, const NpyArray::shape_t& shape) {
    if (shape.size() != 3) {
        return;
    }
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (!(resolution_cm > 0.0)) {
        return;
    }

    const double offset_x_cm = config.offset.x.force_numerical_value_in(cm);
    const double offset_y_cm = config.offset.y.force_numerical_value_in(cm);
    const double offset_z_cm = config.offset.z.force_numerical_value_in(cm);

    const double extent_x_cm = offset_x_cm + resolution_cm * static_cast<double>(shape[0]);
    const double extent_y_cm = offset_y_cm + resolution_cm * static_cast<double>(shape[1]);
    const double extent_z_cm = offset_z_cm + resolution_cm * static_cast<double>(shape[2]);

    double min_x_cm = config.boundaries.min_x.force_numerical_value_in(cm);
    double max_x_cm = config.boundaries.max_x.force_numerical_value_in(cm);
    clampAxis(min_x_cm, max_x_cm, offset_x_cm, extent_x_cm, "x");
    config.boundaries.min_x = min_x_cm * x_extent[cm];
    config.boundaries.max_x = max_x_cm * x_extent[cm];

    double min_y_cm = config.boundaries.min_y.force_numerical_value_in(cm);
    double max_y_cm = config.boundaries.max_y.force_numerical_value_in(cm);
    clampAxis(min_y_cm, max_y_cm, offset_y_cm, extent_y_cm, "y");
    config.boundaries.min_y = min_y_cm * y_extent[cm];
    config.boundaries.max_y = max_y_cm * y_extent[cm];

    double min_z_cm = config.boundaries.min_height.force_numerical_value_in(cm);
    double max_z_cm = config.boundaries.max_height.force_numerical_value_in(cm);
    clampAxis(min_z_cm, max_z_cm, offset_z_cm, extent_z_cm, "height");
    config.boundaries.min_height = min_z_cm * z_extent[cm];
    config.boundaries.max_height = max_z_cm * z_extent[cm];
}

// Throws std::invalid_argument if config.boundaries are degenerate (min >=
// max) on any axis after clampBoundariesToExtent() — i.e. no valid voxels
// remain. Logs the degenerate axis/axes to std::cerr before throwing.
// No-op if the map has no usable resolution (the disjoint check is
// meaningless without one).
void rejectDisjointBoundaries(const types::MapConfig& config) {
    if (!(config.resolution.force_numerical_value_in(cm) > 0.0)) {
        return;
    }

    const bool x_disjoint = config.boundaries.min_x >= config.boundaries.max_x;
    const bool y_disjoint = config.boundaries.min_y >= config.boundaries.max_y;
    const bool z_disjoint = config.boundaries.min_height >= config.boundaries.max_height;

    if (!x_disjoint && !y_disjoint && !z_disjoint) {
        return;
    }

    if (x_disjoint) {
        std::cerr << "Map3DImpl: x boundaries are degenerate after clamping: ["
                  << config.boundaries.min_x.force_numerical_value_in(cm) << ", "
                  << config.boundaries.max_x.force_numerical_value_in(cm) << "] cm.\n";
    }
    if (y_disjoint) {
        std::cerr << "Map3DImpl: y boundaries are degenerate after clamping: ["
                  << config.boundaries.min_y.force_numerical_value_in(cm) << ", "
                  << config.boundaries.max_y.force_numerical_value_in(cm) << "] cm.\n";
    }
    if (z_disjoint) {
        std::cerr << "Map3DImpl: height boundaries are degenerate after clamping: ["
                  << config.boundaries.min_height.force_numerical_value_in(cm) << ", "
                  << config.boundaries.max_height.force_numerical_value_in(cm) << "] cm.\n";
    }
    throw SimulationException(
        "MISSION_BOUNDARY_INVALID",
        "Map3DImpl: boundaries are disjoint from the map's physical extent — no valid "
        "voxels exist after clamping.");
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    // Note: map_ptr is intentionally copied (not moved) into the delegated
    // constructor's argument — inferDefaultConfig() reads map_ptr's shape and
    // must observe it unmoved; argument evaluation order is otherwise
    // unspecified, so a std::move(map_ptr) here would be unsafe.
    : Map3DImpl(map_ptr, inferDefaultConfig(map_ptr)) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    if (!(config_.resolution.force_numerical_value_in(cm) > 0.0)) {
        throw SimulationException("INVALID_RESOLUTION", "Map3DImpl: resolution must be positive.");
    }
    rejectInvalidBoundaries(config_.boundaries);

    if (map_->IsEmpty()) {
        // No data yet (e.g. a freshly created output map): allocate storage
        // sized from the configured boundaries/resolution.
        rejectBoundariesStartingBeforeOffset(config_);
        allocateFreshMap(map_, config_);
    } else {
        validateLoadedArray(*map_);
    }

    clampBoundariesToExtent(config_, map_->Shape());
    rejectDisjointBoundaries(config_);
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isWithinBoundaries(pos, config_.boundaries)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const auto index = computeIndex(pos, config_);
    if (!index) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const auto flat = flatIndex(*index, map_->Shape());
    if (!flat) {
        // Defensive: pos is within the configured boundaries but the backing
        // NpyArray does not have a voxel there (e.g. a misconfigured map).
        return types::VoxelOccupancy::OutOfBounds;
    }

    return rawToOccupancy(readRaw(*map_, *flat));
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    return isWithinBoundaries(pos, config_.boundaries);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    if (!isWithinBoundaries(pos, config_.boundaries)) {
        return;
    }

    const auto index = computeIndex(pos, config_);
    if (!index) {
        return;
    }

    const auto flat = flatIndex(*index, map_->Shape());
    if (!flat) {
        // Defensive: pos is within the configured boundaries but the backing
        // NpyArray does not have a voxel there (e.g. a misconfigured map).
        return;
    }

    writeRaw(*map_, *flat, value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    const std::string path_string = path.string();
    const char* error = map_->SaveNPY(path_string);
    if (error != nullptr) {
        throw std::runtime_error("Failed to save map to '" + path_string + "': " + error);
    }
}

} // namespace simulator
