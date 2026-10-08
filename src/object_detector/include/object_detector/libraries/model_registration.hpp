#pragma once
#include <flann/algorithms/dist.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace object_detector {
namespace registration {
using Point = pcl::PointXYZ;
using Features = pcl::PointCloud<pcl::FPFHSignature33>;
using Alignment = pcl::SampleConsensusPrerejective<Point, Point, pcl::FPFHSignature33>;

// 모델별로 미리 만든 FPFH 특징 kd-tree를 SampleConsensusPrerejective에 재사용한다.
// 모델 상자 밖 점을 건너뛰고 직전 최근접 결과를 재사용하는 ICP 대응 추정이다.
struct CachedAlignment : Alignment {
  void setCachedTargetFeatures(const Features::ConstPtr& features,
                               const FeatureKdTreePtr& tree) {
    target_features_ = features;
    feature_tree_ = tree;
  }
};
struct CertifiedCorrespondences : pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> {
  CertifiedCorrespondences(Eigen::Vector3f minimum, Eigen::Vector3f maximum)
      : minimum(minimum), maximum(maximum) {}
  const Eigen::Vector3f minimum, maximum;
  struct Certificate {
    pcl::PointXYZ origin;
    int nearest = -1;
    double second_distance = 0;
  };
  std::vector<Certificate> certificates;
  void determineCorrespondences(pcl::Correspondences& correspondences, double max_distance) override {
    if (!initCompute()) return;
    certificates.resize(input_->size());
    correspondences.clear();
    correspondences.reserve(indices_->size());
    std::vector<int> neighbors(2);
    std::vector<float> distances(2);
    const double radius = std::abs(max_distance);
    const double max_squared_distance = max_distance * max_distance;
    for (const auto source_index : *indices_) {
      const auto& point = input_->points[source_index];
      const double bound_guard = 32 * std::numeric_limits<float>::epsilon() *
        std::max({1.0, radius, minimum.cast<double>().cwiseAbs().maxCoeff(),
                  maximum.cast<double>().cwiseAbs().maxCoeff(),
                  point.getVector3fMap().cast<double>().cwiseAbs().maxCoeff()});
      bool outside = false;
      for (int axis = 0; axis < 3; ++axis)
        outside = outside || point.data[axis] < minimum[axis] - radius - bound_guard ||
                             point.data[axis] > maximum[axis] + radius + bound_guard;
      // No target within the correspondence radius can lie outside this expanded box.
      if (outside) continue;
      auto& previous = certificates[source_index];
      float squared_distance = std::numeric_limits<float>::infinity();
      bool certified = false;
      if (previous.nearest >= 0) {
        squared_distance = flann::L2_Simple<float>()(point.data, target_->points[previous.nearest].data, 3);
        const Eigen::Vector3d delta = point.getVector3fMap().cast<double>() - previous.origin.getVector3fMap().cast<double>();
        const double guard = 32 * std::numeric_limits<float>::epsilon() *
          std::max({1.0, previous.second_distance, point.getVector3fMap().cast<double>().norm(),
                    previous.origin.getVector3fMap().cast<double>().norm()});
        // Every other target remains at least second_distance - displacement away.
        certified = std::sqrt(static_cast<double>(squared_distance)) + delta.norm() + guard < previous.second_distance;
      }
      if (!certified) {
        const int count = tree_->nearestKSearch(point, 2, neighbors, distances);
        if (count < 1) continue;
        const double second = count > 1 ? std::sqrt(static_cast<double>(distances[1]))
                                        : std::numeric_limits<double>::infinity();
        if (count > 1 && distances[0] == distances[1]) tree_->nearestKSearch(point, 1, neighbors, distances);
        previous.origin = point;
        previous.nearest = neighbors[0];
        previous.second_distance = second;
        squared_distance = distances[0];
      }
      if (squared_distance <= max_squared_distance)
        correspondences.emplace_back(source_index, previous.nearest, squared_distance);
    }
    deinitCompute();
  }
};
}  // namespace registration
}  // namespace object_detector
