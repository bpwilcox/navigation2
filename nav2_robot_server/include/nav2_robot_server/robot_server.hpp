// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_ROBOT_SERVER__ROBOT_SERVER_HPP_
#define NAV2_ROBOT_SERVER__ROBOT_SERVER_HPP_

#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "nav2_msgs/srv/get_robot_pose.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"

namespace nav2_robot_server
{

class RobotServer : public nav2_util::LifecycleNode
{
public:
  RobotServer();
  ~RobotServer();

protected:
  // The lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  rclcpp::Service<nav2_msgs::srv::GetRobotPose>::SharedPtr get_robot_pose_service_;
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id);

  void get_robot_pose_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetRobotPose::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetRobotPose::Response> response);

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_robot_server

#endif  // NAV2_ROBOT_SERVER__ROBOT_SERVER_HPP_
