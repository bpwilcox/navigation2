/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dave Hershberger
*********************************************************************/
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

namespace dwb_critics
{

class TestCollisionChecker : public CollisionChecker
{
public:
  TestCollisionChecker(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
  : CollisionChecker(node, costmap_sub, footprint_sub),
    node_(node),
    layers_("frame", false, false),
    new_pose_received_(true)
  {
    node_->get_parameter_or<std::string>("global_frame", global_frame_, std::string("map"));

    tf2_ros::Buffer tf(node_->get_clock());

    // Add Static Layer
    nav2_costmap_2d::StaticLayer * slayer = new nav2_costmap_2d::StaticLayer();
    layers_.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(slayer));
    slayer->initialize(&layers_, "static", &tf, node_);

    // Add Inflation Layer
    nav2_costmap_2d::InflationLayer * ilayer = new nav2_costmap_2d::InflationLayer();
    ilayer->initialize(&layers_, "inflation", &tf, node_);
    std::shared_ptr<nav2_costmap_2d::Layer> ipointer(ilayer);
    layers_.addPlugin(ipointer);

    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", std::bind(&TestCollisionChecker::onPoseReceived, this, std::placeholders::_1));

    footprint_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      footprint_sub_->getTopicName(), rmw_qos_profile_default);

    costmap_pub_ = new nav2_costmap_2d::Costmap2DPublisher(node_,
        layers_.getCostmap(), global_frame_, costmap_sub_->getTopicName(), true);
  }

  ~TestCollisionChecker() {}

  void setFootprint(double footprint_padding, std::string footprint)
  {
    if (footprint != "" && footprint != "[]") {
      std::vector<geometry_msgs::msg::Point> new_footprint;
      if (nav2_costmap_2d::makeFootprintFromString(footprint, new_footprint)) {
        nav2_costmap_2d::padFootprint(new_footprint, footprint_padding);
        footprint_ = new_footprint;
        layers_.setFootprint(footprint_);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid footprint string");
      }
    }
  }

  void setFootprint(double footprint_padding, double robot_radius)
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    new_footprint = nav2_costmap_2d::makeFootprintFromRadius(robot_radius);
    nav2_costmap_2d::padFootprint(new_footprint, footprint_padding);
    footprint_ = new_footprint;
    layers_.setFootprint(footprint_);
  }

  void setFootprint(std::vector<geometry_msgs::msg::Point> polygon)
  {
    footprint_ = polygon;
    layers_.setFootprint(footprint_);
  }

  void setPose(double x, double y, double yaw)
  {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
  }

  void publish()
  {
    auto timer_callback = [this]() -> void 
    {
      publishFootprint();
      publishCostmap();
      RCLCPP_INFO(node_->get_logger(), "Collision Free?: %d", isCollisionFree(current_pose_));
      // if (new_pose_received_) {
      //   RCLCPP_INFO(node_->get_logger(), "Collision Free?: %d", isCollisionFree(current_pose_));
      //   new_pose_received_ = false;
      // }
    };

    timer_ = node_->create_wall_timer(1s, timer_callback); 
  }

  std::vector<geometry_msgs::msg::Point> setRadii(
    double length, double width)
  {
    std::vector<geometry_msgs::msg::Point> polygon;
    geometry_msgs::msg::Point p;
    p.x = width;
    p.y = length;
    polygon.push_back(p);
    p.x = width;
    p.y = -length;
    polygon.push_back(p);
    p.x = -width;
    p.y = -length;
    polygon.push_back(p);
    p.x = -width;
    p.y = length;
    polygon.push_back(p);
    
    return polygon;
  }

protected:
  void publishFootprint()
  {
    geometry_msgs::msg::PolygonStamped oriented_footprint;
    oriented_footprint.header.frame_id = global_frame_;
    oriented_footprint.header.stamp = node_->now();
    nav2_costmap_2d::transformFootprint(x_, y_, yaw_, footprint_, oriented_footprint);
    footprint_pub_->publish(oriented_footprint);
  }

  void publishCostmap()
  {
    layers_.updateMap(x_, y_, yaw_);
    costmap_pub_->publishCostmap();
  }

  void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    current_pose_.x = x_;
    current_pose_.y = y_;
    current_pose_.theta = yaw_;
    new_pose_received_ = true;
  }

  double x_, y_, yaw_;
  rclcpp::Node::SharedPtr node_;
  nav2_costmap_2d::LayeredCostmap layers_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  nav2_costmap_2d::Costmap2DPublisher * costmap_pub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string global_frame_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::Pose2D current_pose_;
  bool new_pose_received_;
};

}  // namespace dwb_critics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // This node is created and spun in order to pass clock to buffer for Costmap2DROS node
  auto node = rclcpp::Node::make_shared(
    "test_collision_checker", nav2_util::get_node_options_default());

  std::string costmap_topic;
  std::string footprint_topic;
  double x, y, yaw;
  double robot_radius, footprint_padding;

  node->declare_parameter("costmap_topic");
  node->declare_parameter("footprint_topic");
  node->declare_parameter("robot_radius");
  node->declare_parameter("footprint_padding");
  node->declare_parameter("pose_x");
  node->declare_parameter("pose_y");
  node->declare_parameter("pose_theta");

  node->get_parameter_or<std::string>("costmap_topic", costmap_topic, "costmap");
  node->get_parameter_or<std::string>("footprint_topic", footprint_topic, "footprint"); 
  node->get_parameter_or<double>("robot_radius", robot_radius, 1.0);
  node->get_parameter_or<double>("footprint_padding", footprint_padding, 0.0);
  node->get_parameter_or<double>("pose_x", x, 0);
  node->get_parameter_or<double>("pose_y", y, 0);
  node->get_parameter_or<double>("pose_theta", yaw, 0);

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  try
  {
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(node, footprint_topic);
    dwb_critics::TestCollisionChecker MyTest(node, costmap_sub_, footprint_sub_);
    MyTest.setPose(x,y,yaw);
    // MyTest.setFootprint(footprint_padding, robot_radius);
    MyTest.setFootprint(MyTest.setRadii(1.0, 1.0));
    MyTest.publish();
    rclcpp::spin(node);
  }
  catch(std::runtime_error& e)
  {
    RCLCPP_ERROR(node->get_logger(),"%s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}


