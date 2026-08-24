#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_322889890_315113738 {

class ScanResultToVoxels {
public:
    // Applies a LiDAR scan directly to the output map.
    // Writes only Occupied, Empty and PotentiallyOccupied states.
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_322889890_315113738