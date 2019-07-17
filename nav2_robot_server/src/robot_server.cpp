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

#include "nav2_robot_server/robot_server.hpp"

#include <string>

#include "rclcpp/qos.hpp"
#include "urdf/model.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/duration_conversions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::placeholders;

namespace nav2_robot_server
{

RobotServer::RobotServer()
: LifecycleNode("robot_server", "", true, nav2_util::get_node_options_default())
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));

}

RobotServer::~RobotServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
RobotServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("robot_base_frame", robot_base_frame_);

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, rclcpp_node_, false);

  get_robot_pose_service_ = create_service<nav2_msgs::srv::GetRobotPose>("GetPose",
      std::bind(&RobotServer::get_robot_pose_callback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");



  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  tf_buffer_.reset();
  tf_listener_.reset();
  get_robot_pose_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotServer::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void RobotServer::get_robot_pose_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetRobotPose::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetRobotPose::Response> response)
{
  response->is_pose_valid = getRobotPose(response->pose, "map");
}

bool
RobotServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped robot_pose;

  tf2::toMsg(tf2::Transform::getIdentity(), pose.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time();

  // Save time for checking tf delay later
  rclcpp::Time current_time = now();

  // Get the global pose of the robot
  try {
    tf_buffer_->transform(robot_pose, pose, frame_id);
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(get_logger(),
      "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }

  // Check global_pose timeout
  // TODO(bpwilcox): use toSec() function in more recent rclcpp branch
  if (current_time - pose.header.stamp >
    nav2_util::duration_from_seconds(transform_tolerance_))
  {
    RCLCPP_WARN(
      get_logger(),
      "Transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f, difference: %.4f", //NOLINT
      tf2::timeToSec(tf2_ros::fromMsg(current_time)),
      tf2::timeToSec(tf2_ros::fromMsg(pose.header.stamp)),
      transform_tolerance_,
      tf2::timeToSec(tf2_ros::fromMsg(current_time)) -
      tf2::timeToSec(tf2_ros::fromMsg(pose.header.stamp)));

    return false;
  }

  return true;
}

}  // namespace nav2_robot_server
