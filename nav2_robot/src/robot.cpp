// Copyright (c) 2018 Intel Corporation
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

#include "nav2_robot/robot.hpp"

#include <string>
#include <exception>
#include "urdf/model.h"

namespace nav2_robot
{

Robot::Robot(rclcpp::Node::SharedPtr & node)
: node_(node), initial_pose_received_(false), initial_odom_received_(false)
{
  // TODO(mhpanah): Topic names for pose and odom should should be configured with parameters
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&Robot::onPoseReceived, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", std::bind(&Robot::onOdomReceived, this, std::placeholders::_1));

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  node_->get_parameter_or<std::string>("global_frame", global_frame_, std::string("map"));
  node_->get_parameter_or<std::string>("robot_base_frame", robot_base_frame_, std::string("base_link"));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

void
Robot::onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // TODO(mjeronimo): serialize access
  current_pose_ = msg;
  if (!initial_pose_received_) {
    initial_pose_received_ = true;
  }
}

void
Robot::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
  if (!initial_odom_received_) {
    initial_odom_received_ = true;
  }
}

bool
Robot::getGlobalLocalizerPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  if (!initial_pose_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Robot: Can't return current pose: Initial pose not yet received.");
    return false;
  }

  robot_pose = current_pose_;
  return true;
}

// TODO(mhpanah): We should get current pose from transforms.
bool
Robot::getCurrentPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  return getGlobalLocalizerPose(robot_pose);
}

bool
Robot::getOdometry(nav_msgs::msg::Odometry::SharedPtr & robot_odom)
{
  if (!initial_odom_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Robot: Can't return current velocity: Initial odometry not yet received.");
    return false;
  }

  robot_odom = current_odom_;
  return true;
}

// TODO(mhpanah): modify this method name and implementation to include robot types and Serial #(ID)
std::string
Robot::getName()
{
  // Temporarily just returning a string until we enable parsing URDF file.
  return "turtlebot";
}

void
Robot::sendVelocity(geometry_msgs::msg::Twist twist)
{
  vel_pub_->publish(twist);
}

bool
Robot::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time();

  rclcpp::Time current_time = node_->now();  // save time for checking tf delay later
  // get the global pose of the robot
  try {
    tf_buffer_->transform(robot_pose, global_pose, global_frame_);
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout

  return true;
}

}  // namespace nav2_robot
