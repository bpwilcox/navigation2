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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/testing_helper.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestCollisionChecker : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCollisionChecker(std::string name)
  : Costmap2DROS(name),
    costmap_received_(false)
  {
    std::string costmap_topic = "costmap_raw";
    std::string footprint_topic = "footprint";

    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      rclcpp_node_, costmap_topic);
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      rclcpp_node_, footprint_topic);
    collision_checker_ = std::make_unique<nav2_costmap_2d::CollisionChecker>(
      rclcpp_node_, costmap_sub_, footprint_sub_, *tf_buffer_);
  }

  void initialize()
  {
    on_configure(get_current_state());
    base_rel_map_.transform = tf2::toMsg(tf2::Transform::getIdentity());
    base_rel_map_.child_frame_id = "base_link";
    base_rel_map_.header.frame_id = "map";
    base_rel_map_.header.stamp = rclcpp_node_->now();
    tf_buffer_->setTransform(base_rel_map_, "collision_checker_test");
    transform_tolerance_ = 100;
    on_activate(get_current_state());
  }
  // nav2_util::CallbackReturn
  // on_configure(const rclcpp_lifecycle::State & /*state*/)
  // {
  //   RCLCPP_INFO(get_logger(), "Configuring");

  //   std::string costmap_topic = "costmap_raw";
  //   std::string footprint_topic = "footprint";

  //   costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
  //     rclcpp_node_, costmap_topic);
  //   footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
  //     rclcpp_node_, footprint_topic);
  //   collision_checker_ = std::make_unique<nav2_costmap_2d::CollisionChecker>(
  //     rclcpp_node_, costmap_sub_, footprint_sub_, *tf_buffer_);

  //   base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
  //   base_rel_map.child_frame_id = "base_link";
  //   base_rel_map.header.frame_id = "map";
  //   base_rel_map.header.stamp = now();
  //   tf_buffer_->setTransform(base_rel_map, "collision_checker_test");

  //   return nav2_util::CallbackReturn::SUCCESS;
  // }

  // nav2_util::CallbackReturn
  // on_activate(const rclcpp_lifecycle::State & /*state*/)
  // {
  //   RCLCPP_INFO(get_logger(), "Activating");

  //   costmap_pub_->on_activate();
  //   footprint_pub_->on_activate();

  //   auto timer_callback = [this]() -> void
  //     {
  //       try {
  //         costmap_sub_->getCostmap();
  //         costmap_received_ = true;
  //       } catch (const std::runtime_error & e) {
  //         costmap_received_ = false;
  //       }
  //       publishFootprint();
  //       publishCostmap();
  //     };

  //   timer_ = create_wall_timer(0.1s, timer_callback);

  //   return nav2_util::CallbackReturn::SUCCESS;
  // }

  // nav2_util::CallbackReturn
  // on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  // {
  //   RCLCPP_INFO(get_logger(), "Deactivating");

  //   costmap_pub_->on_deactivate();
  //   footprint_pub_->on_deactivate();
  //   timer_->cancel();

  //   return nav2_util::CallbackReturn::SUCCESS;
  // }

  // nav2_util::CallbackReturn
  // on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  // {
  //   RCLCPP_INFO(get_logger(), "Cleaning Up");
  //   delete layers_;
  //   layers_ = nullptr;

  //   timer_->reset();
  //   tf_buffer_.reset();
  //   tf_listener_.reset();

  //   footprint_sub_.reset();
  //   footprint_pub_.reset();

  //   costmap_sub_.reset();
  //   costmap_pub_.reset();

  //   return nav2_util::CallbackReturn::SUCCESS;
  // }

  ~TestCollisionChecker() {}

  bool testPose(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    setPose(x, y, theta);
    costmap_received_ = false;

    while (!costmap_received_) {
      try {
        costmap_sub_->getCostmap();
        costmap_received_ = true;
      } catch (const std::runtime_error & e) {
        costmap_received_ = false;
      }      
    }

    return collision_checker_->isCollisionFree(pose);
  }

  void setFootprint(double footprint_padding, double robot_radius)
  {
    footprint_padding_ = footprint_padding;
    setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius));
  }

protected:
  void setPose(double x, double y, double theta)
  {
    x_ = x;
    y_ = y;
    yaw_ = theta;

    geometry_msgs::msg::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    pose.orientation = tf2::toMsg(q);

    tf2::Transform transform;
    tf2::fromMsg(pose, transform);
    base_rel_map_.transform = tf2::toMsg(transform);
    base_rel_map_.header.stamp = rclcpp_node_->now();
    tf_buffer_->setTransform(base_rel_map_, "collision_checker_test");
  }
 
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::CollisionChecker> collision_checker_;
  double x_, y_, yaw_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::string global_frame_;
  bool costmap_received_;
  geometry_msgs::msg::TransformStamped base_rel_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    collision_checker_ = std::make_shared<TestCollisionChecker>("test_collision_checker");
    collision_checker_->initialize();
  }

  ~TestNode()
  {
    collision_checker_->on_deactivate(collision_checker_->get_current_state());
    collision_checker_->on_cleanup(collision_checker_->get_current_state());
  }

protected:
  std::shared_ptr<TestCollisionChecker> collision_checker_;
};

TEST_F(TestNode, uknownSpace)
{
  collision_checker_->setFootprint(0, 1);

  // Completely off map
  ASSERT_EQ(collision_checker_->testPose(5, 13, 0), false);

  // Partially off map
  ASSERT_EQ(collision_checker_->testPose(5, 9.5, 0), false);

  // In unknown region inside map
  ASSERT_EQ(collision_checker_->testPose(2, 4, 0), false);
}

TEST_F(TestNode, FreeSpace)
{
  collision_checker_->setFootprint(0, 1);

  // In complete free space
  ASSERT_EQ(collision_checker_->testPose(2, 8.5, 0), true);

  // Partially in inscribed space
  ASSERT_EQ(collision_checker_->testPose(2.5, 7, 0), true);
}

TEST_F(TestNode, CollisionSpace)
{
  collision_checker_->setFootprint(0, 1);

  // Completely in obstacle
  ASSERT_EQ(collision_checker_->testPose(8.5, 6.5, 0), false);

  // Partially in obstacle
  ASSERT_EQ(collision_checker_->testPose(4.5, 4.5, 0), false);
}
