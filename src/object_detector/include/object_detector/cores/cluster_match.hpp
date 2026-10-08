#pragma once
#include <object_detector/libraries/model_registration.hpp>
#include <object_detector/libraries/npc_models.hpp>
#include <interfaces/Objects.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <utility>

namespace object_detector {
struct ClusterMatchParameters {
  double voxel_leaf_m = 0.08;
  double cluster_tolerance_m = 0.45;
  int min_cluster_points = 6;
  int max_cluster_points = 100000;
  double ground_distance_m = 0.06;
  double ground_max_tilt_rad = 0.35;
  double ground_seed_max_z_m = -0.25;
  double ground_cell_m = 0.5;
  double ground_region_m = 5.0;
  int ground_min_inliers = 50;
  double normal_radius_m = 0.3;
  double feature_radius_m = 0.5;
  int registration_iterations = 1024;
  int icp_iterations = 40;
  double correspondence_distance_m = 0.15;
  double min_inlier_fraction = 0.85;
  double max_rmse_m = 0.06;
  double ambiguity_score_margin = 0.10;
  double min_pose_shape_ratio = 0.002;
  double model_max_tilt_rad = 0.35;
  bool exclude_ego = true;
  std::array<double, 3> ego_min_m{{-0.79, -1.080418, -0.3685}};
  std::array<double, 3> ego_max_m{{3.845008, 1.080417, 1.275546}};
  // 파라미터 로더가 models_file 자산에서 읽은 NPC 모델이다.
  std::vector<ModelTemplate> models;
};

// base_link 점군을 군집화하고, 각 군집을 NPC 모델에 정합해 라벨·collider 상자를 붙인다.
class ClusterMatch {
 public:
  explicit ClusterMatch(ClusterMatchParameters parameters) : parameters_(std::move(parameters)) {
    auto models = std::move(parameters_.models);
    parameters_.models.clear();
    for (auto& model : models) {
      if (model.points.empty()) continue;
      auto surface = voxelize(model.points.makeShared(), parameters_.voxel_leaf_m);
      auto descriptors = describe(surface);
      pcl::search::KdTree<Point>::Ptr surface_tree(new pcl::search::KdTree<Point>);
      surface_tree->setInputCloud(surface);
      pcl::search::KdTree<Point>::Ptr descriptor_tree(new pcl::search::KdTree<Point>);
      if (!descriptors.points->empty()) descriptor_tree->setInputCloud(descriptors.points);
      Alignment::FeatureKdTreePtr feature_tree(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
      if (!descriptors.features->empty()) feature_tree->setInputCloud(descriptors.features);
      Eigen::Vector3f mean = Eigen::Vector3f::Zero();
      float min_x = std::numeric_limits<float>::infinity(), max_x = -min_x;
      Eigen::Vector3f minimum = Eigen::Vector3f::Constant(min_x), maximum = -minimum;
      for (const auto& point : *surface) {
        mean += point.getVector3fMap();
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        for (int axis = 0; axis < 3; ++axis) {
          minimum[axis] = std::min(minimum[axis], point.data[axis]);
          maximum[axis] = std::max(maximum[axis], point.data[axis]);
        }
      }
      mean /= surface->size();
      const auto box = observedBox(*surface);
      const float yaw = 2 * std::atan2(box.bbox_pose.orientation.z, box.bbox_pose.orientation.w);
      models_.push_back({model.semantic_label.empty() ? std::move(model.label) : std::move(model.semantic_label),
                        surface, std::move(descriptors),
                        model.collider_available, model.collider_pose, model.collider_size,
                        surface_tree, descriptor_tree, feature_tree, mean, yaw, min_x, max_x, minimum, maximum});
    }
  }

  interfaces::Objects process(const sensor_msgs::PointCloud2& cloud) {
    auto points = readPoints(cloud);
    interfaces::Objects output;
    if (points->empty()) return output;
    points = voxelize(points, parameters_.voxel_leaf_m);
    points = removeGround(points);
    if (points->empty()) return output;
    // VLP16's 2-degree vertical ray spacing grows with range; keep XY separation.
    Cloud::Ptr search_points(new Cloud(*points));
    for (auto& point : *search_points) {
      const double vertical_gap_m = std::hypot(point.x, point.y) * std::tan(0.03490658503988659);
      point.z /= std::max(1.0, 1.1 * vertical_gap_m / parameters_.cluster_tolerance_m);
    }
    typename pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(search_points);
    pcl::EuclideanClusterExtraction<Point> extraction;
    extraction.setInputCloud(search_points);
    extraction.setSearchMethod(tree);
    extraction.setClusterTolerance(parameters_.cluster_tolerance_m);
    extraction.setMinClusterSize(parameters_.min_cluster_points);
    extraction.setMaxClusterSize(parameters_.max_cluster_points);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    for (const auto& indices : clusters) {
      Cloud::Ptr cluster(new Cloud);
      pcl::copyPointCloud(*points, indices.indices, *cluster);
      auto object = observedBox(*cluster);
      classify(cluster, object);
      output.objects.push_back(std::move(object));
    }
    return output;
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;
  using Features = pcl::PointCloud<pcl::FPFHSignature33>;
  using Alignment = registration::Alignment;
  using CachedAlignment = registration::CachedAlignment;
  using CertifiedCorrespondences = registration::CertifiedCorrespondences;
  struct Descriptors {
    Cloud::Ptr points;
    Features::Ptr features;
  };
  struct CachedModel {
    std::string label;
    Cloud::Ptr surface;
    Descriptors descriptors;
    bool collider_available;
    geometry_msgs::Pose collider_pose;
    geometry_msgs::Vector3 collider_size;
    pcl::search::KdTree<Point>::Ptr surface_tree;
    pcl::search::KdTree<Point>::Ptr descriptor_tree;
    Alignment::FeatureKdTreePtr feature_tree;
    Eigen::Vector3f mean;
    float yaw;
    float min_x;
    float max_x;
    Eigen::Vector3f minimum, maximum;
  };

  Cloud::Ptr readPoints(const sensor_msgs::PointCloud2& cloud) const {
    Cloud::Ptr points(new Cloud);
    points->reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    const auto offset = [&](const char* name) {
      return std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::PointField& field) { return field.name == name; })->offset;
    };
    const std::uint32_t offsets[] = {offset("x"), offset("y"), offset("z")};
    for (std::uint32_t row = 0; row < cloud.height; ++row)
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto* record = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step + col * cloud.point_step;
        float xyz[3];
        for (std::size_t axis = 0; axis < 3; ++axis)
          std::memcpy(&xyz[axis], record + offsets[axis], sizeof(float));
        bool inside_ego = parameters_.exclude_ego;
        for (std::size_t axis = 0; axis < 3; ++axis)
          inside_ego = inside_ego && xyz[axis] >= parameters_.ego_min_m[axis] && xyz[axis] <= parameters_.ego_max_m[axis];
        if (!inside_ego) points->emplace_back(xyz[0], xyz[1], xyz[2]);
      }
    return points;
  }

  Cloud::Ptr voxelize(const Cloud::ConstPtr& input, double leaf_m) const {
    Cloud::Ptr output(new Cloud);
    pcl::VoxelGrid<Point> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(leaf_m, leaf_m, leaf_m);
    filter.filter(*output);
    return output;
  }

  Cloud::Ptr removeGround(const Cloud::Ptr& points) const {
    // Equal-area lowest-point support prevents dense obstacle tops dominating RANSAC.
    std::map<std::pair<int, int>, Point> lowest;
    for (const auto& point : *points) {
      if (point.z > parameters_.ground_seed_max_z_m) continue;
      const auto cell = std::make_pair(
        static_cast<int>(std::floor(point.x / parameters_.ground_cell_m)),
        static_cast<int>(std::floor(point.y / parameters_.ground_cell_m)));
      const auto entry = lowest.find(cell);
      if (entry == lowest.end() || point.z < entry->second.z) lowest[cell] = point;
    }
    Cloud::Ptr seeds(new Cloud);
    for (const auto& cell : lowest) seeds->push_back(cell.second);
    if (seeds->size() < static_cast<std::size_t>(parameters_.ground_min_inliers)) return points;
    Cloud::Ptr seed_xy(new Cloud(*seeds));
    for (auto& point : *seed_xy) point.z = 0;
    pcl::search::KdTree<Point> seed_tree;
    seed_tree.setInputCloud(seed_xy);
    std::map<std::pair<int, int>, std::vector<std::size_t>> regions;
    for (std::size_t i = 0; i < points->size(); ++i) {
      const auto& point = points->at(i);
      regions[{static_cast<int>(std::floor(point.x / parameters_.ground_region_m)),
               static_cast<int>(std::floor(point.y / parameters_.ground_region_m))}].push_back(i);
    }
    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(parameters_.ground_max_tilt_rad);
    segmentation.setDistanceThreshold(parameters_.ground_distance_m);
    segmentation.setMaxIterations(200);
    Cloud::Ptr objects(new Cloud);
    for (const auto& region : regions) {
      // Overlapping local support follows road height without discarding thin objects.
      const Point center((region.first.first + 0.5) * parameters_.ground_region_m,
                         (region.first.second + 0.5) * parameters_.ground_region_m, 0);
      std::vector<int> neighbors;
      std::vector<float> distances;
      seed_tree.radiusSearch(center, parameters_.ground_region_m, neighbors, distances);
      if (neighbors.size() < static_cast<std::size_t>(parameters_.ground_min_inliers)) {
        // Clipped observed regions can require adjacent road-cell support.
        neighbors.clear();
        distances.clear();
        seed_tree.radiusSearch(center, 2 * parameters_.ground_region_m, neighbors, distances);
      }
      Cloud::Ptr support(new Cloud);
      pcl::copyPointCloud(*seeds, neighbors, *support);
      pcl::PointIndices inliers;
      pcl::ModelCoefficients plane;
      if (support->size() >= static_cast<std::size_t>(parameters_.ground_min_inliers)) {
        segmentation.setInputCloud(support);
        segmentation.segment(inliers, plane);
      }
      const bool supported = inliers.indices.size() >= static_cast<std::size_t>(parameters_.ground_min_inliers);
      for (const auto index : region.second) {
        const auto& point = points->at(index);
        const auto& c = plane.values;
        if (!supported || std::abs(c[0]*point.x + c[1]*point.y + c[2]*point.z + c[3]) > parameters_.ground_distance_m)
          objects->push_back(point);
      }
    }
    return objects;
  }

  interfaces::Object observedBox(const Cloud& cluster) const {
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& point : cluster) mean += Eigen::Vector2d(point.x, point.y);
    mean /= cluster.size();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto& point : cluster) {
      const Eigen::Vector2d delta = Eigen::Vector2d(point.x, point.y) - mean;
      covariance += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Vector2d axis = solver.eigenvectors().col(1);
    if (axis.x() < 0) axis = -axis;
    const double yaw = std::atan2(axis.y(), axis.x());
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d upper = -lower;
    for (const auto& point : cluster) {
      const Eigen::Vector3d local = rotation.transpose() * Eigen::Vector3d(point.x, point.y, point.z);
      lower = lower.cwiseMin(local);
      upper = upper.cwiseMax(local);
    }
    const Eigen::Vector3d center = rotation * ((lower + upper) * 0.5);
    const Eigen::Vector3d size = upper - lower;
    interfaces::Object object;
    object.label = "unknown";
    object.bbox_pose.position.x = center.x();
    object.bbox_pose.position.y = center.y();
    object.bbox_pose.position.z = center.z();
    object.position = object.bbox_pose.position;
    object.bbox_pose.orientation.z = std::sin(yaw * 0.5);
    object.bbox_pose.orientation.w = std::cos(yaw * 0.5);
    object.bbox_size.x = size.x();
    object.bbox_size.y = size.y();
    object.bbox_size.z = size.z();
    return object;
  }

  Descriptors describe(const Cloud::Ptr& points) const {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<Point, pcl::Normal> estimation;
    estimation.setInputCloud(points);
    estimation.setRadiusSearch(parameters_.normal_radius_m);
    estimation.compute(*normals);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (const auto& point : *points) center += point.getVector3fMap();
    center /= points->size();
    for (std::size_t i = 0; i < points->size(); ++i)
      if (normals->at(i).getNormalVector3fMap().dot(points->at(i).getVector3fMap() - center) < 0)
        normals->at(i).getNormalVector3fMap() *= -1;
    Features all_features;
    pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(points);
    fpfh.setInputNormals(normals);
    fpfh.setRadiusSearch(parameters_.feature_radius_m);
    fpfh.compute(all_features);
    Descriptors output{Cloud::Ptr(new Cloud), Features::Ptr(new Features)};
    for (std::size_t i = 0; i < all_features.size(); ++i) {
      // Isolated neighbourhoods can leave computed normals or FPFH undefined.
      bool defined = normals->at(i).getNormalVector3fMap().allFinite();
      for (const auto value : all_features[i].histogram) defined = defined && std::isfinite(value);
      if (defined) {
        output.points->push_back(points->at(i));
        output.features->push_back(all_features[i]);
      }
    }
    return output;
  }

  void classify(const Cloud::Ptr& cluster, interfaces::Object& object) const {
    if (models_.empty()) return;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& point : *cluster) mean += point.getVector3fMap().cast<double>();
    mean /= cluster->size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& point : *cluster) {
      const Eigen::Vector3d delta = point.getVector3fMap().cast<double>() - mean;
      covariance += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> support(covariance);
    // Near-planar observations can fit multiple full-object poses with low residual.
    if (support.eigenvalues()[0] <= parameters_.min_pose_shape_ratio * support.eigenvalues()[2]) return;
    const auto source = describe(cluster);
    const auto coarse = voxelize(cluster, 3.0 * parameters_.voxel_leaf_m);
    const float source_yaw = 2 * std::atan2(object.bbox_pose.orientation.z, object.bbox_pose.orientation.w);
    double best_score = -std::numeric_limits<double>::infinity();
    double second_score = best_score;
    const CachedModel* best_model = nullptr;
    bool best_eligible = false;
    Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
    struct PoseSeed { bool available = false; Eigen::Matrix4f inverse_pose = Eigen::Matrix4f::Identity(); };
    struct ModelFit { double score = -std::numeric_limits<double>::infinity(); Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); bool eligible = false; };
    std::vector<PoseSeed> feature_seeds(models_.size());
    // Keep feature registration in model order because PCL consumes the shared RNG.
    for (std::size_t i = 0; i < models_.size(); ++i) {
      const auto& model = models_[i];
      CachedAlignment alignment;
      const bool descriptors_available = source.points->size() >= 3 && model.descriptors.points->size() >= 3;
      if (descriptors_available) {
        alignment.setInputSource(source.points);
        alignment.setSourceFeatures(source.features);
        alignment.setInputTarget(model.descriptors.points);
        alignment.setSearchMethodTarget(model.descriptor_tree, true);
        alignment.setCachedTargetFeatures(model.descriptors.features, model.feature_tree);
        alignment.setMaximumIterations(parameters_.registration_iterations);
        alignment.setNumberOfSamples(3);
        alignment.setCorrespondenceRandomness(std::min<std::size_t>(5, model.descriptors.points->size()));
        alignment.setSimilarityThreshold(0.9f);
        alignment.setMaxCorrespondenceDistance(parameters_.correspondence_distance_m);
        alignment.setInlierFraction(parameters_.min_inlier_fraction);
        Cloud aligned;
        alignment.align(aligned);
      }
      feature_seeds[i].available = descriptors_available && alignment.hasConverged();
      if (feature_seeds[i].available) feature_seeds[i].inverse_pose = alignment.getFinalTransformation();
    }
    std::vector<ModelFit> fits(models_.size());
    const auto fit_model = [&](std::size_t i) {
      const auto& model = models_[i];
      const bool feature_pose = feature_seeds[i].available;
      double model_score = -std::numeric_limits<double>::infinity();
      Eigen::Matrix4f model_pose = Eigen::Matrix4f::Identity();
      bool model_eligible = false;
      const int yaw_seeds = std::min(16, parameters_.registration_iterations);
      const int seeds = 3 * yaw_seeds + (feature_pose ? 1 : 0);
      for (int seed = 0; seed < seeds; ++seed) {
        Eigen::Matrix4f inverse_pose = Eigen::Matrix4f::Identity();
        const bool feature_seed = feature_pose && seed == 0;
        const int yaw_seed = seed - (feature_pose ? 1 : 0);
        if (feature_seed) {
          inverse_pose = feature_seeds[i].inverse_pose;
        } else {
          inverse_pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(
            model.yaw - source_yaw + (yaw_seed / 3) * 6.283185307179586f / yaw_seeds,
            Eigen::Vector3f::UnitZ()).toRotationMatrix();
          inverse_pose.block<3, 1>(0, 3) = model.mean - inverse_pose.block<3, 3>(0, 0) * mean.cast<float>();
          // A partial visible length can share either endpoint rather than the full-model centroid.
          const int anchor = yaw_seed % 3;
          if (anchor != 0) {
            float lower = std::numeric_limits<float>::infinity(), upper = -lower;
            for (const auto& point : *cluster) {
              const float x = (inverse_pose.block<3, 3>(0, 0) * point.getVector3fMap()).x();
              lower = std::min(lower, x);
              upper = std::max(upper, x);
            }
            inverse_pose(0, 3) = anchor == 1 ? model.min_x - lower : model.max_x - upper;
          }
        }
        Cloud aligned;
        bool converged = true;
        // Coarse correspondences bootstrap partial surfaces without descriptor-density assumptions.
        for (double distance : {std::max(1.0, parameters_.correspondence_distance_m),
                                std::max(0.5, parameters_.correspondence_distance_m),
                                std::max(0.3, parameters_.correspondence_distance_m),
                                parameters_.correspondence_distance_m}) {
          if (feature_seed && distance != parameters_.correspondence_distance_m) continue;
          // PCL convergence history must not leak between pose trials or distance scales.
          pcl::IterativeClosestPoint<Point, Point> refinement;
          refinement.setCorrespondenceEstimation(pcl::registration::CorrespondenceEstimationBase<Point, Point>::Ptr(new CertifiedCorrespondences(model.minimum, model.maximum)));
          refinement.setInputSource(distance == parameters_.correspondence_distance_m || coarse->size() < 3
                                      ? cluster : coarse);
          refinement.setInputTarget(model.surface);
          refinement.setSearchMethodTarget(model.surface_tree, true);
          refinement.setMaximumIterations(parameters_.icp_iterations);
          refinement.setTransformationEpsilon(1e-8);
          refinement.setEuclideanFitnessEpsilon(1e-8);
          refinement.setMaxCorrespondenceDistance(distance);
          refinement.align(aligned, inverse_pose);
          if (!refinement.hasConverged()) {
            converged = false;
            break;
          }
          inverse_pose = refinement.getFinalTransformation();
        }
        if (!converged) continue;
        const Eigen::Matrix4f pose = inverse_pose.inverse();
        if (pose(2, 2) < std::cos(parameters_.model_max_tilt_rad)) continue;
        std::vector<int> neighbours(1);
        std::vector<float> distances(1);
        std::size_t inliers = 0;
        double squared_error = 0;
        const double max_squared_distance = parameters_.correspondence_distance_m * parameters_.correspondence_distance_m;
        // Score observed points only: unobserved model surfaces are not mismatches.
        for (const auto& point : *cluster) {
          const Eigen::Vector4f local = inverse_pose * point.getVector4fMap();
          const Point query(local.x(), local.y(), local.z());
          if (model.surface_tree->nearestKSearch(query, 1, neighbours, distances) == 1 && distances[0] <= max_squared_distance) {
            ++inliers;
            squared_error += distances[0];
          }
        }
        if (inliers == 0) continue;
        const double fraction = static_cast<double>(inliers) / cluster->size();
        const double rmse = std::sqrt(squared_error / inliers);
        // Near-threshold alternatives still compete; excluding them can manufacture confidence.
        const bool eligible = fraction >= parameters_.min_inlier_fraction && rmse <= parameters_.max_rmse_m;
        const double score = fraction - rmse / parameters_.correspondence_distance_m;
        if (score > model_score) {
          model_score = score;
          model_pose = pose;
          model_eligible = eligible;
        }
      }
      fits[i] = {model_score, model_pose, model_eligible};
    };
    for (std::size_t i = 0; i < models_.size(); ++i) fit_model(i);
    // Reduce in the original model order to preserve equal scores and semantic margins.
    for (std::size_t i = 0; i < models_.size(); ++i) {
      const auto& model = models_[i];
      const auto model_score = fits[i].score;
      const auto& model_pose = fits[i].pose;
      const bool model_eligible = fits[i].eligible;
      if (model_score > best_score) {
        if (!best_model || model.label != best_model->label) second_score = best_score;
        best_score = model_score;
        best_model = &model;
        best_pose = model_pose;
        best_eligible = model_eligible;
      } else if (!best_model || model.label != best_model->label) {
        second_score = std::max(second_score, model_score);
      }
    }
    if (!best_model || !best_eligible || best_score - second_score < parameters_.ambiguity_score_margin) return;
    object.label = best_model->label;
    if (!best_model->collider_available) return;
    const auto& local_pose = best_model->collider_pose;
    const Eigen::Vector3d local_center(local_pose.position.x, local_pose.position.y, local_pose.position.z);
    const Eigen::Matrix3d rotation = best_pose.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d center = rotation * local_center + best_pose.block<3, 1>(0, 3).cast<double>();
    Eigen::Quaterniond orientation = Eigen::Quaterniond(rotation) *
      Eigen::Quaterniond(local_pose.orientation.w, local_pose.orientation.x,
                         local_pose.orientation.y, local_pose.orientation.z);
    orientation.normalize();
    if (orientation.w() < 0) orientation.coeffs() *= -1;
    object.bbox_pose.position.x = center.x();
    object.bbox_pose.position.y = center.y();
    object.bbox_pose.position.z = center.z();
    object.position = object.bbox_pose.position;
    object.bbox_pose.orientation.x = orientation.x();
    object.bbox_pose.orientation.y = orientation.y();
    object.bbox_pose.orientation.z = orientation.z();
    object.bbox_pose.orientation.w = orientation.w();
    object.bbox_size = best_model->collider_size;
  }

  ClusterMatchParameters parameters_;
  std::vector<CachedModel> models_;
};
}  // namespace object_detector
