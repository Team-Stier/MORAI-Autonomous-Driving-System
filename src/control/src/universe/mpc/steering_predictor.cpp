// Copyright 2023 The Autoware Foundation
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

#include "control/universe/mpc/steering_predictor.hpp"

namespace autoware { namespace motion { namespace control { namespace mpc_lateral_controller {

SteeringPredictor::SteeringPredictor(const double steer_tau, const double steer_delay)
: m_steer_tau(steer_tau), m_input_delay(steer_delay)
{
}

double SteeringPredictor::calcSteerPrediction()
{
  auto t_start = m_time_prev;
  auto t_end = m_current_time;
  m_time_prev = t_end;

  const double duration = (t_end - t_start).toSec();
  const double time_constant = m_steer_tau;

  const double initial_response = std::exp(-duration / time_constant) * m_steer_prediction_prev;

  if (m_ctrl_cmd_vec.size() <= 2) {
    setPrevResult(initial_response);
    return initial_response;
  }

  const auto predicted_steering = initial_response + getSteerCmdSum(t_start, t_end, time_constant);
  setPrevResult(predicted_steering);

  return predicted_steering;
}

void SteeringPredictor::storeSteerCmd(const double steer)
{
  const auto time_delayed = m_current_time + ros::Duration(m_input_delay);
  Lateral cmd;
  cmd.stamp = time_delayed;
  cmd.steering_tire_angle = static_cast<float>(steer);

  // store published ctrl cmd
  m_ctrl_cmd_vec.emplace_back(cmd);

  if (m_ctrl_cmd_vec.size() <= 2) {
    return;
  }

  // remove unused ctrl cmd
  constexpr double store_time = 0.3;
  if ((time_delayed - m_ctrl_cmd_vec.at(1).stamp).toSec() > m_input_delay + store_time) {
    m_ctrl_cmd_vec.erase(m_ctrl_cmd_vec.begin());
  }
}

double SteeringPredictor::getSteerCmdSum(
  const ros::Time & t_start, const ros::Time & t_end, const double time_constant) const
{
  if (m_ctrl_cmd_vec.size() <= 2) {
    return 0.0;
  }

  // Find first index of control command container
  size_t idx = 1;
  while (t_start > ros::Time(m_ctrl_cmd_vec.at(idx).stamp)) {
    if ((idx + 1) >= m_ctrl_cmd_vec.size()) {
      return 0.0;
    }
    ++idx;
  }

  // Compute steer command input response
  double steer_sum = 0.0;
  auto t = t_start;
  while (t_end > ros::Time(m_ctrl_cmd_vec.at(idx).stamp)) {
    const double duration = (ros::Time(m_ctrl_cmd_vec.at(idx).stamp) - t).toSec();
    t = ros::Time(m_ctrl_cmd_vec.at(idx).stamp);
    steer_sum += (1 - std::exp(-duration / time_constant)) *
                 static_cast<double>(m_ctrl_cmd_vec.at(idx - 1).steering_tire_angle);
    ++idx;
    if (idx >= m_ctrl_cmd_vec.size()) {
      break;
    }
  }

  const double duration = (t_end - t).toSec();
  steer_sum += (1 - std::exp(-duration / time_constant)) *
               static_cast<double>(m_ctrl_cmd_vec.at(idx - 1).steering_tire_angle);

  return steer_sum;
}

void SteeringPredictor::setPrevResult(const double & steering)
{
  m_steer_prediction_prev = steering;
}

}}}}  // namespace autoware::motion::control::mpc_lateral_controller
