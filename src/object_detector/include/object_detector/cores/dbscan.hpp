#pragma once
#include <interfaces/Objects.h>
#include <object_detector/libraries/minimum_area_obb.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

namespace object_detector {

struct DbscanParameters {
  // #C5_PARAMETERS
  // 이 구조체의 초기값은 원본 synthetic 회귀 fixture와의 호환을 유지한다.
  // 실제 C5 계수는 parameters/dbscan.yaml을 ParamReader로 전부 읽어 생성자에 주입한다.
  // voxel_leaf_size는 3축 voxel 변 길이[m], eps는 3D 이웃 반경[m]이다.
  // MinPts는 검색점 자신도 세며 cluster 크기 제한은 voxel 처리 이후 점 개수다.
  // C5: voxel=0.075, eps=0.16, MinPts=8, min/max cluster=6/1000.
  // #ROI_INTEGRATION
  // x=[0,20], y=[-5,5], z=[0.2,2]m는 base_link_level의 임시 Fixed ROI다.
  // 최종 RDDF ROI Core는 다른 담당자가 개발하며 이 고정 crop은 그 구현이 아니다.
  // 현재 순서는 Fixed ROI → VoxelGrid → DBSCAN → OBB로 crop은 여기서 한 번만 한다.
  // ROI Core를 앞 Stage에 넣을 때는 담당자·리드와 입력 계약을 합의하고,
  // 이 crop의 제거·비활성화 방법을 함께 정해 서로 다른 ROI를 중복 적용하지 않는다.
  double roi_x_min = 0.0;
  double roi_x_max = 20.0;
  double roi_y_min = -5.0;
  double roi_y_max = 5.0;
  double roi_z_min = 0.2;
  double roi_z_max = 2.0;
  double voxel_leaf_size = 0.04;
  double dbscan_eps = 0.18;
  int dbscan_min_points = 4;
  int min_cluster_size = 5;
  int max_cluster_size = 1000;
};

// #DBSCAN
// base_link_level 점군을 임시 Fixed ROI로 자른 뒤 VoxelGrid의 centroid로 줄인다.
// KD-tree radius search의 3D 반경 eps에 자기 자신을 포함한 MinPts를 적용한다.
// 밀도 충분한 core point만 확장하고 앞서 Noise였던 점도 border로 편입한다.
// enqueue 시 Assigned로 바꿔 같은 점이 다른 cluster에 중복 배정되지 않게 한다.
// voxel 이후 cluster 크기를 min/max로 필터링하고 최소 면적 OBB를 한 개씩 만든다.
// tracking·속도·가속도 추정은 하지 않으며 미추정 availability는 false로 남는다.
// mounting과 roll/pitch는 upstream 계산을 신뢰하며 Core는 입력 frame을 바꾸지 않는다.
// 반환은 Objects payload뿐이고 stamp·frame 주입은 최신 공통 Pipeline의 책임이다.
// 새 ROI Stage를 추가할 때는 위 #ROI_INTEGRATION의 단일 crop 계약을 먼저 정한다.
class Dbscan {
 public:
  explicit Dbscan(DbscanParameters parameters) : parameters_(parameters) {}

  // #FRAME_CONTRACT
  // 이 Core는 입력 PointCloud2의 좌표를 다른 frame으로 변환하지 않는다.
  // production 입력은 roll/pitch가 제거된 base_link_level이므로 ROI 범위,
  // VoxelGrid의 cell, DBSCAN 거리·cluster point와 OBB center·orientation은
  // 모두 그 수평 frame 기준이다. mounting TF와 자세 보정은 upstream에서
  // 이미 적용되었으므로 여기서 다시 적용하지 않는다.
  // OBB yaw는 base_link_level의 +x/+y 평면 기준이며 bbox_size는 OBB 자체
  // local 축의 전체 길이이다. payload만 반환하고 Pipeline이 같은 계산 frame과
  // 입력 stamp를 최종 /objects 헤더에 주입한다. header 이름만 base_link로
  // 변경하거나 결과를 최신 자세로 역회전하면 좌표와 frame 계약이 어긋난다.
  // 다른 frame이 필요한 downstream이 자신의 경계에서 시각에 맞게 변환한다.
  interfaces::Objects process(const sensor_msgs::PointCloud2& cloud) {
    Cloud xyz;
    pcl::fromROSMsg(cloud, xyz);
    const auto filtered = downsample(cropRoi(xyz));
    interfaces::Objects output;
    if (filtered->empty()) return output;

    for (const auto& cluster : findClusters(filtered)) {
      const auto size = static_cast<int>(cluster.size());
      if (size >= parameters_.min_cluster_size && size <= parameters_.max_cluster_size)
        output.objects.push_back(bbox::makeObb(*filtered, cluster));
    }
    return output;
  }

 private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using Tree = pcl::KdTreeFLANN<pcl::PointXYZ>;
  enum class PointState { Unvisited, Noise, Assigned };

  Cloud::Ptr cropRoi(const Cloud& cloud) const {
    Cloud::Ptr cropped(new Cloud);
    for (const auto& point : cloud) {
      if (point.x >= parameters_.roi_x_min && point.x <= parameters_.roi_x_max &&
          point.y >= parameters_.roi_y_min && point.y <= parameters_.roi_y_max &&
          point.z >= parameters_.roi_z_min && point.z <= parameters_.roi_z_max)
        cropped->push_back(point);
    }
    return cropped;
  }

  Cloud::Ptr downsample(const Cloud::Ptr& cloud) const {
    Cloud::Ptr filtered(new Cloud);
    if (cloud->empty()) return filtered;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const auto leaf = static_cast<float>(parameters_.voxel_leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    return filtered;
  }

  std::vector<int> neighborsWithin(Tree& tree, int index) const {
    std::vector<int> neighbors;
    std::vector<float> squared_distances;
    // PCL searches the indexed point at distance zero too: MinPts includes self.
    tree.radiusSearch(index, parameters_.dbscan_eps, neighbors, squared_distances);
    return neighbors;
  }

  std::vector<int> expandCluster(Tree& tree, int seed, std::vector<int> neighbors,
                                 std::vector<PointState>& states) const {
    std::vector<int> cluster{seed};
    states[seed] = PointState::Assigned;
    for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
      if (cursor != 0) neighbors = neighborsWithin(tree, cluster[cursor]);
      // Only core points expand; border points remain in the cluster.
      if (static_cast<int>(neighbors.size()) < parameters_.dbscan_min_points) continue;
      for (const auto index : neighbors) {
        if (states[index] == PointState::Assigned) continue;
        // Earlier noise can become a border point. Assign on enqueue to avoid duplicates.
        states[index] = PointState::Assigned;
        cluster.push_back(index);
      }
    }
    return cluster;
  }

  std::vector<std::vector<int>> findClusters(const Cloud::Ptr& cloud) const {
    Tree tree;
    tree.setInputCloud(cloud);
    std::vector<PointState> states(cloud->size(), PointState::Unvisited);
    std::vector<std::vector<int>> clusters;
    for (std::size_t index = 0; index < cloud->size(); ++index) {
      if (states[index] != PointState::Unvisited) continue;
      auto neighbors = neighborsWithin(tree, static_cast<int>(index));
      if (static_cast<int>(neighbors.size()) < parameters_.dbscan_min_points) {
        states[index] = PointState::Noise;
        continue;
      }
      clusters.push_back(expandCluster(tree, static_cast<int>(index), neighbors, states));
    }
    return clusters;
  }


  DbscanParameters parameters_;
};
}  // namespace object_detector
