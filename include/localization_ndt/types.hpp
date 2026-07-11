#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace localization_ndt {

using PointT      = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

}  // namespace localization_ndt
