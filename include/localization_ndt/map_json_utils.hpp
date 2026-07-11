#pragma once

#include <string>
#include <vector>

#include "localization_ndt/types.hpp"

namespace localization_ndt {

struct JsonMapHeader {
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  double resolution = 0.05;
  double size_w = 0.0; // meters
  double size_h = 0.0; // meters
};

// Load JSON map and build point cloud from occupyPoints.
bool LoadPointCloudFromJson(const std::string& json_path,
                            PointCloudT::Ptr* out_cloud,
                            JsonMapHeader* out_h,
                            std::string* out_err);

}  // namespace localization_ndt
