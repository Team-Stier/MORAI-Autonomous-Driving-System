#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace object_detector {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Clusters = std::vector<Cloud>;
}  // namespace object_detector
