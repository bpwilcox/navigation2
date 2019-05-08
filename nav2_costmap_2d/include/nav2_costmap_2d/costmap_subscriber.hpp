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

#ifndef NAV2_COSTMAP_2D__COSTMAP_2D_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_2D_SUBSCRIBER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_costmap_2d
{
/**
 * @class CostmapSubscriber
 * @brief A tool to subscribe to costmap messages
 * */
class CostmapSubscriber
{
public:
  /**
   * @brief  Constructor for the CostmapSubscriber
   */
  CostmapSubscriber(
    rclcpp::Node::SharedPtr ros_node,
    std::string & topic_name);

  /**
   * @brief  Destructor
   */
  ~CostmapSubscriber();

Costmap2D * getCostmap();

protected:
  void toCostmap2D(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  Costmap2D * costmap_;
  std::string topic_name_;
  bool costmap_received_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
};

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_SUBSCRIBER_HPP_
