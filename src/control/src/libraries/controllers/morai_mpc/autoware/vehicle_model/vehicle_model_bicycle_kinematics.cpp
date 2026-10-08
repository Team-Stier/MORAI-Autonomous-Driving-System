/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "control/libraries/tracking/morai_mpc/autoware/vehicle_model/vehicle_model_bicycle_kinematics.h"
#include <stdexcept>

namespace control { namespace morai_mpc { namespace autoware {

KinematicsBicycleModel::KinematicsBicycleModel(const double &wheelbase, const double &steer_lim, const double &steer_tau, double steer_gain)
    : VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2)
{
    wheelbase_ = wheelbase;
    steer_lim_ = steer_lim;
    steer_tau_ = steer_tau;
    if (!std::isfinite(steer_gain) || steer_gain < .5 || steer_gain > 1.5)
        throw std::invalid_argument("Invalid effective steering gain");
    steer_gain_ = steer_gain;
};

void KinematicsBicycleModel::calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                                                     Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt)
{
    auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

    /* Linearize delta around delta_r (referece delta) */
    double delta_r = atan(wheelbase_ * curvature_);
    if (abs(delta_r) >= steer_lim_)
        delta_r = steer_lim_ * (double)sign(delta_r);
    double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(steer_tau_) || steer_tau_ <= 0.0)
        throw std::invalid_argument("Invalid steering integration interval");
    // Exact zero-order hold of this triangular continuous model. Bilinear
    // discretization has an artificial negative actuator pole when dt > 2*tau.
    const double decay = std::exp(-dt / steer_tau_);
    const double integral1 = -steer_tau_ * std::expm1(-dt / steer_tau_);
    const double integral2 = steer_tau_ * (dt - integral1);
    const double yaw_gain = velocity_ / wheelbase_ * cos_delta_r_squared_inv;
    Ad << 1.0, velocity_ * dt, velocity_ * yaw_gain * integral2,
        0.0, 1.0, yaw_gain * integral1,
        0.0, 0.0, decay;
    Bd << velocity_ * yaw_gain * steer_gain_ * (0.5 * dt * dt - integral2),
        yaw_gain * steer_gain_ * (dt - integral1),
        steer_gain_ * (1.0 - decay);

    Cd << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0;

    const double yaw_bias = -velocity_ * curvature_ + velocity_ / wheelbase_ *
        (tan(delta_r) - delta_r * cos_delta_r_squared_inv);
    Wd << 0.5 * velocity_ * yaw_bias * dt * dt, yaw_bias * dt, 0.0;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    Uref(0, 0) = std::atan(wheelbase_ * curvature_) / steer_gain_;
}

}}}  // namespace control::morai_mpc::autoware
