/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_plugins/kinematic_parameters.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"

using std::fabs;

using nav_2d_utils::moveDeprecatedParameter;
namespace dwb_plugins
{

KinematicParameters::KinematicParameters()
{
}

void KinematicParameters::initialize(const nav2_util::LifecycleNode::SharedPtr & /* nh */)
{

  parameter_client_= std::make_shared<nav2_util::ParametersClient>("/parameter_blackboard");

  // TODO(bpwilcox): Support deprecated parameters with parameter client?
  // // Special handling for renamed parameters
  // moveDeprecatedParameter<double>(nh, "max_vel_theta", "max_rot_vel");
  // moveDeprecatedParameter<double>(nh, "min_speed_xy", "min_trans_vel");
  // moveDeprecatedParameter<double>(nh, "max_speed_xy", "max_trans_vel");
  // moveDeprecatedParameter<double>(nh, "min_speed_theta", "min_rot_vel");

  min_vel_x_ = parameter_client_->get_parameter<double>("min_vel_x", 0.0);
  min_vel_y_ = parameter_client_->get_parameter<double>("min_vel_y", 0.0);
  max_vel_x_ = parameter_client_->get_parameter<double>("max_vel_x", 0.0);
  max_vel_y_ = parameter_client_->get_parameter<double>("max_vel_y", 0.0);
  max_vel_theta_ = parameter_client_->get_parameter<double>("max_vel_theta", 0.0);
  min_speed_xy_ = parameter_client_->get_parameter<double>("min_speed_xy", 0.0);
  max_speed_xy_ = parameter_client_->get_parameter<double>("max_speed_xy", 0.0);
  min_speed_theta_ = parameter_client_->get_parameter<double>("min_speed_theta", 0.0);
  acc_lim_x_ = parameter_client_->get_parameter<double>("acc_lim_x", 0.0);
  acc_lim_y_ = parameter_client_->get_parameter<double>("acc_lim_y", 0.0);
  acc_lim_theta_ = parameter_client_->get_parameter<double>("acc_lim_theta", 0.0);
  decel_lim_x_ = parameter_client_->get_parameter<double>("decel_lim_x", 0.0);
  decel_lim_y_ = parameter_client_->get_parameter<double>("decel_lim_y", 0.0);
  decel_lim_theta_ = parameter_client_->get_parameter<double>("decel_lim_theta", 0.0);

  min_speed_xy_sq_ = min_speed_xy_ * min_speed_xy_;
  max_speed_xy_sq_ = max_speed_xy_ * max_speed_xy_;
}

bool KinematicParameters::isValidSpeed(double x, double y, double theta)
{
  double vmag_sq = x * x + y * y;
  if (max_speed_xy_ >= 0.0 && vmag_sq > max_speed_xy_sq_) {return false;}
  if (min_speed_xy_ >= 0.0 && vmag_sq < min_speed_xy_sq_ &&
    min_speed_theta_ >= 0.0 && fabs(theta) < min_speed_theta_) {return false;}
  if (vmag_sq == 0.0 && theta == 0.0) {return false;}
  return true;
}

}  // namespace dwb_plugins
