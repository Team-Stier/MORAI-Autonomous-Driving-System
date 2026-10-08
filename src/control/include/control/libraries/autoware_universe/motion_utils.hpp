// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <control/libraries/autoware_universe/geometry.hpp>
namespace autoware { namespace motion_utils {
template <class T>
void validateNonEmpty(const T & points)
{
  if (points.empty()) {

    throw std::invalid_argument("[autoware_motion_utils] validateNonEmpty(): Points is empty.");
  }
}

template <class T>
[[nodiscard]] boost::optional<bool> isDrivingForward(const T & points, const double min_dist = 0.01)
{
  if (points.size() < 2) {
    return boost::none;
  }

  const auto & first_pose = autoware_utils::get_pose(points.at(0));

  // Find the target point that is at least min_dist away from the first point
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & target_pose = autoware_utils::get_pose(points.at(i));
    if (autoware_utils::calc_distance2d(first_pose, target_pose) >= min_dist) {
      return autoware_utils::is_driving_forward(first_pose, target_pose);
    }
  }

  // All points are within min_dist, cannot determine direction reliably
  return boost::none;
}

template <class T>
[[nodiscard]] T removeOverlapPoints(const T & points, const size_t start_idx = 0)
{
  if (points.size() < start_idx + 1) {
    return points;
  }

  T dst;
  dst.reserve(points.size());

  for (size_t i = 0; i <= start_idx; ++i) {
    dst.push_back(points.at(i));
  }

  constexpr double eps = 1.0E-08;
  for (size_t i = start_idx + 1; i < points.size(); ++i) {
    const auto prev_p = autoware_utils::get_point(dst.back());
    const auto curr_p = autoware_utils::get_point(points.at(i));
    if (std::abs(prev_p.x - curr_p.x) < eps && std::abs(prev_p.y - curr_p.y) < eps) {
      continue;
    }
    dst.push_back(points.at(i));
  }

  return dst;
}

template <class T>
[[nodiscard]] boost::optional<size_t> searchZeroVelocityIndex(
  const T & points_with_twist, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points_with_twist);

  constexpr double epsilon = 1e-3;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (std::fabs(points_with_twist.at(i).longitudinal_velocity_mps) < epsilon) {
      return i;
    }
  }

  return {};
}

template <class T>
[[nodiscard]] boost::optional<size_t> searchZeroVelocityIndex(
  const T & points_with_twist, const size_t src_idx)
{
  validateNonEmpty(points_with_twist);

  return searchZeroVelocityIndex(points_with_twist, src_idx, points_with_twist.size());
}

template <class T>
[[nodiscard]] boost::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist)
{
  return searchZeroVelocityIndex(points_with_twist, 0, points_with_twist.size());
}

template <class T>
[[nodiscard]] size_t findNearestIndex(const T & points, const geometry_msgs::Point & point)
{
  validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware_utils::calc_squared_distance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

template <class T>
double calcLongitudinalOffsetToSegment(
  const T & points, const size_t seg_idx, const geometry_msgs::Point & p_target,
  const bool throw_exception = false)
{
  if (seg_idx >= points.size() - 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Failed to calculate longitudinal offset because the given segment index is out of the "
      "points size.");

    if (throw_exception) {
      throw std::out_of_range(error_message);
    }
    
    return std::nan("");
  }

  if (throw_exception) {
    validateNonEmpty(points);
  } else {
    try {
      validateNonEmpty(points);
    } catch (const std::exception & e) {
      
      return std::nan("");
    }
  }

  // Equivalent to removeOverlapPoints(points, seg_idx) but allocation-free: indices up to seg_idx
  // are kept as-is, and the back point is the first point after seg_idx that is not coincident
  // with the point at seg_idx (the previously kept point). Only the two points at the resulting
  // overlap-removed indices seg_idx and seg_idx + 1 are needed here.
  const auto p_front = autoware_utils::get_point(points.at(seg_idx));
  constexpr double overlap_eps = 1.0E-08;
  size_t back_idx = seg_idx + 1;
  while (back_idx < points.size()) {
    const auto candidate = autoware_utils::get_point(points.at(back_idx));
    if (
      std::abs(p_front.x - candidate.x) >= overlap_eps ||
      std::abs(p_front.y - candidate.y) >= overlap_eps) {
      break;
    }
    ++back_idx;
  }

  if (back_idx >= points.size()) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Longitudinal offset calculation is not supported for the same points.");

    if (throw_exception) {
      throw std::runtime_error(error_message);
    }
    
    return std::nan("");
  }

  const auto p_back = autoware_utils::get_point(points.at(back_idx));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

template <class T>
size_t findNearestSegmentIndex(const T & points, const geometry_msgs::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += autoware_utils::calc_distance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::Point & src_point, const size_t dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  {  // with dist and yaw thresholds
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        autoware_utils::calc_squared_distance2d(points.at(i), pose.position);
      const auto yaw = autoware_utils::calc_yaw_deviation(
        autoware_utils::get_pose(points.at(i)), pose);

      if (squared_dist_threshold < squared_dist || yaw_threshold < std::abs(yaw)) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  {  // with dist threshold
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        autoware_utils::calc_squared_distance2d(points.at(i), pose.position);

      if (squared_dist_threshold < squared_dist) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  {  // with yaw threshold
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto yaw = autoware_utils::calc_yaw_deviation(
        autoware_utils::get_pose(points.at(i)), pose);

      if (yaw_threshold < std::abs(yaw)) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      const auto squared_dist =
        autoware_utils::calc_squared_distance2d(points.at(i), pose.position);
      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  // without any threshold
  return findNearestIndex(points, pose.position);
}

template <class T>
size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const T & points, const geometry_msgs::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  // find first nearest index with soft constraints (not segment index)
  const size_t nearest_idx =
    findFirstNearestIndexWithSoftConstraints(points, pose, dist_threshold, yaw_threshold);

  // calculate segment index
  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, pose.position);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}
}}
