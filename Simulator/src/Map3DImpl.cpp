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

constexpr std::size_t kNumAxes = 3;

using VoxelIndex = std::array<long, kNumAxes>;

// Converts a world position into voxel indices using the formula:
//   idx = floor((world_coordinate + offset) / resolution)
// Returns std::nullopt if the map has no usable resolution (e.g. a default
// MapConfig), since indices cannot be derived in that case.
[[nodiscard]] std::optional<VoxelIndex> computeIndex(const Position3D& pos,
                                                       const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (!(resolution_cm > 0.0)) {
        return std::nullopt;
    }

    const double rel_x = (pos.x + config.offset.x).force_numerical_value_in(cm);
    const double rel_y = (pos.y + config.offset.y).force_numerical_value_in(cm);
    const double rel_z = (pos.z + config.offset.z).force_numerical_value_in(cm);

    return VoxelIndex{
        static_cast<long>(std::floor(rel_x / resolution_cm)),
        static_cast<long>(std::floor(rel_y / resolution_cm)),
        static_cast<long>(std::floor(rel_z / resolution_cm)),
    };
}

// Checks whether `pos` is inside the configured map bounds using
// half-open intervals (min <= coordinate < max).
// NpyArray index validity is checked separately by flatIndex().
[[nodiscard]] bool isWithinBoundaries(const Position3D& pos, const types::MappingBounds& bounds) {
    return pos.x >= bounds.min_x && pos.x < bounds.max_x &&
           pos.y >= bounds.min_y && pos.y < bounds.max_y &&
           pos.z >= bounds.min_height && pos.z < bounds.max_height;
}

// Converts [X, Y, Z] voxel indices to a flat NpyArray index.
// Returns std::nullopt if the indices are outside the actual array shape.
[[nodiscard]] std::optional<std::size_t> flatIndex(const VoxelIndex& index,
                                                     const NpyArray::shape_t& shape) {
    if (shape.size() != kNumAxes) {
        return std::nullopt;
    }
    for (std::size_t axis = 0; axis < kNumAxes; ++axis) {
        if (index[axis] < 0 || static_cast<std::size_t>(index[axis]) >= shape[axis]) {
            return std::nullopt;
        }
    }
    const auto x = static_cast<std::size_t>(index[0]);
    const auto y = static_cast<std::size_t>(index[1]);
    const auto z = static_cast<std::size_t>(index[2]);
    return x * shape[1] * shape[2] + y * shape[2] + z;
}

// Reads a voxel value from any supported NpyArray dtype.
// The value is normalized to long before converting it to VoxelOccupancy.
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

// Converts a stored raw value to VoxelOccupancy.
// OutOfBounds is never stored in the array.
// Positive values greater than 1 are Occupied; invalid negatives are rejected.
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
            if (raw > kRawOccupied) {
                return types::VoxelOccupancy::Occupied;
            }

            throw SimulationException(
                "INVALID_MAP_FORMAT",
                "Map3DImpl: invalid negative voxel value.");
            }
}

// Converts VoxelOccupancy to its raw representation and writes it to the NpyArray.
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

// Creates the backing NpyArray for a new map using the configured size and resolution.
// All voxels are initialized to Unmapped.
void allocateFreshMap(std::shared_ptr<NpyArray>& map_ptr, const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);

    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    if (resolution_cm > 0.0) {
        const auto& bounds = config.boundaries;
        const auto& offset = config.offset;
        nx = axisVoxelCount((bounds.max_x + offset.x).force_numerical_value_in(cm), resolution_cm);
        ny = axisVoxelCount((bounds.max_y + offset.y).force_numerical_value_in(cm), resolution_cm);
        nz = axisVoxelCount((bounds.max_height + offset.z).force_numerical_value_in(cm),
                             resolution_cm);
    }

    map_ptr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(int),
                                          NpyArray::GetTypeChar(typeid(int)));
    map_ptr->Allocate();
    std::fill_n(map_ptr->Data<int>(), map_ptr->NumValue(), kRawUnmapped);
}

// Validates the shape, memory layout and dtype of an NpyArray loaded from disk.
// Throws INVALID_MAP_FORMAT if the array is not supported.
void validateLoadedArray(const NpyArray& array) {
    if (array.Shape().size() != kNumAxes) {
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

// Default resolution used when inferring MapConfig from an NpyArray.
constexpr double kDefaultResolutionCm = 1.0;

// Fallback for construction without an explicit MapConfig:
// infers the config from the NpyArray, assuming 1cm resolution and zero offset.
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

// Rejects bounds where min >= max on any axis.
// This check runs before any clamping to the map extent.
void rejectInvalidBoundaries(const types::MappingBounds& bounds) {
    if (bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y &&
        bounds.min_height < bounds.max_height) {
        return;
    }
    throw SimulationException("MISSION_BOUNDARY_INVALID",
                              "Map3DImpl: invalid mapping boundaries: min must be smaller than max.");
}

// Rejects fresh-map configs whose boundaries start before the map offset,
// since those positions would map to negative voxel indices.
void rejectBoundariesStartingBeforeOffset(const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);

    const double min_x_index =
        std::floor((config.boundaries.min_x + config.offset.x).force_numerical_value_in(cm) /
                    resolution_cm);
    const double min_y_index =
        std::floor((config.boundaries.min_y + config.offset.y).force_numerical_value_in(cm) /
                    resolution_cm);
    const double min_z_index =
        std::floor((config.boundaries.min_height + config.offset.z).force_numerical_value_in(cm) /
                    resolution_cm);

    if (min_x_index >= 0.0 && min_y_index >= 0.0 && min_z_index >= 0.0) {
        return;
    }
    throw SimulationException("MISSION_BOUNDARY_INVALID",
                              "Map3DImpl: configured boundaries start before the map offset.");
}

// Clamps one configured boundary axis to the physical range covered by the NpyArray.
// Logs if the configured bounds had to be adjusted.
void clampAxis(double& configured_min_cm, double& configured_max_cm, double map_min_cm, double map_max_cm,
               const char* axis_name) {
    const double original_min = configured_min_cm;
    const double original_max = configured_max_cm;

    if (configured_min_cm < map_min_cm) {
        configured_min_cm = map_min_cm;
    }
    if (configured_max_cm > map_max_cm) {
        configured_max_cm = map_max_cm;
    }

    if (configured_min_cm != original_min || configured_max_cm != original_max) {
        std::cerr << "Map3DImpl: clamped " << axis_name << " mapping boundary from ["
                  << original_min << ", " << original_max << "] cm to [" << configured_min_cm << ", "
                  << configured_max_cm << "] cm to fit the map extent.\n";
    }
}

// Clamps config.boundaries so that every position within them maps to a
// valid index into `shape`. No-op if the map has no usable resolution.
void clampBoundariesToExtent(types::MapConfig& config, const NpyArray::shape_t& shape) {
    if (shape.size() != kNumAxes) {
        return;
    }
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (!(resolution_cm > 0.0)) {
        return;
    }

    const double offset_x_cm = config.offset.x.force_numerical_value_in(cm);
    const double offset_y_cm = config.offset.y.force_numerical_value_in(cm);
    const double offset_z_cm = config.offset.z.force_numerical_value_in(cm);

    // Physical (mission-relative) position of voxel index 0 and of voxel index shape[axis] --
    // a voxel-to-position conversion, so it subtracts the offset.
    const double map_min_x_cm = -offset_x_cm;
    const double map_min_y_cm = -offset_y_cm;
    const double map_min_z_cm = -offset_z_cm;
    const double extent_x_cm = resolution_cm * static_cast<double>(shape[0]) - offset_x_cm;
    const double extent_y_cm = resolution_cm * static_cast<double>(shape[1]) - offset_y_cm;
    const double extent_z_cm = resolution_cm * static_cast<double>(shape[2]) - offset_z_cm;

    double min_x_cm = config.boundaries.min_x.force_numerical_value_in(cm);
    double max_x_cm = config.boundaries.max_x.force_numerical_value_in(cm);
    clampAxis(min_x_cm, max_x_cm, map_min_x_cm, extent_x_cm, "x");
    config.boundaries.min_x = min_x_cm * x_extent[cm];
    config.boundaries.max_x = max_x_cm * x_extent[cm];

    double min_y_cm = config.boundaries.min_y.force_numerical_value_in(cm);
    double max_y_cm = config.boundaries.max_y.force_numerical_value_in(cm);
    clampAxis(min_y_cm, max_y_cm, map_min_y_cm, extent_y_cm, "y");
    config.boundaries.min_y = min_y_cm * y_extent[cm];
    config.boundaries.max_y = max_y_cm * y_extent[cm];

    double min_z_cm = config.boundaries.min_height.force_numerical_value_in(cm);
    double max_z_cm = config.boundaries.max_height.force_numerical_value_in(cm);
    clampAxis(min_z_cm, max_z_cm, map_min_z_cm, extent_z_cm, "height");
    config.boundaries.min_height = min_z_cm * z_extent[cm];
    config.boundaries.max_height = max_z_cm * z_extent[cm];
}

// Rejects bounds that have no valid range left after clamping to the map extent.
// Logs the affected axes before throwing MISSION_BOUNDARY_INVALID.
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


// map_ptr is intentionally not moved here because inferDefaultConfig()
// must still read it before delegating to the main constructor.
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(map_ptr, inferDefaultConfig(map_ptr)) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig& map_config)
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
        // Defensive: configured bounds may still disagree with the actual array shape.
        return types::VoxelOccupancy::OutOfBounds;
    }

    const long raw = readRaw(*map_, *flat);

    if (raw > kRawOccupied && !non_standard_positive_reported_) {
        std::cerr
            << "Map3DImpl: non-standard positive voxel value encountered; "
            "values greater than 1 are treated as Occupied.\n";

        non_standard_positive_reported_ = true;
    }

    return rawToOccupancy(raw);
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
        // Defensive: configured bounds may still disagree with the actual array shape.
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
