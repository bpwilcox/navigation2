/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  rclcpp::Node::SharedPtr ros_node,
  std::string topic_name)
: node_(ros_node), topic_name_(topic_name)
{
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(topic_name,
      std::bind(&CostmapSubscriber::costmap_callback, this, std::placeholders::_1));
}

CostmapSubscriber::~CostmapSubscriber() {}

bool CostmapSubscriber::getCostmap(Costmap2D & costmap)
{
  if (!costmap_received_) {
    return false;
  }
  costmap = Costmap2D(costmap_);
  return true;
}

void CostmapSubscriber::toCostmap2D(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  Costmap2D costmap = Costmap2D(
    msg->info.width, msg->info.height,
    msg->info.resolution, msg->info.origin.position.x,
    msg->info.origin.position.y);
  
  unsigned char * master_array = costmap.getCharMap();
  master_array = msg->data;

  costmap_ = Costmap2D(costmap);
}

void CostmapSubscriber::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  toCostmap2D(msg);
  if (!costmap_received_) {
    costmap_received_ = true;
  }
}

}  // namespace nav2_costmap_2d

