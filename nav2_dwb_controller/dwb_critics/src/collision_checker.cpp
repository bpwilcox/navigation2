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


#include "dwb_critics/collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "dwb_critics/line_iterator.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace dwb_critics
{

Footprint CollisionChecker::getOrientedFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const Footprint & footprint_spec)
{
  std::vector<geometry_msgs::msg::Point> oriented_footprint;
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    new_pt.x = pose.x + footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th;
    new_pt.y = pose.y + footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th;
  }
  return oriented_footprint;
}

CollisionChecker::CollisionChecker(
  rclcpp::Node::SharedPtr ros_node,
  std::string costmap_topic,
  std::string footprint_topic,
  std::string name)
: node_(ros_node), costmap_topic_(costmap_topic),
  footprint_topic_(footprint_topic), name_(name)
{
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node_, costmap_topic_);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(node_, footprint_topic_);
  costmap_ = costmap_sub_->getCostmap();
}

CollisionChecker::~CollisionChecker() {}

bool CollisionChecker::isCollisionFree(
const geometry_msgs::msg::Pose2D & pose)
{
  if (scorePose(pose) < 0) {
    return false;
  }
  return true;
}

double CollisionChecker::scorePose(
const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }

  Footprint unoriented_footprint;
  if (!footprint_sub_->getFootprint(unoriented_footprint)) {
    throw nav_core2::PlannerException("Footprint not available.");
  }

  Footprint footprint = getOrientedFootprint(pose, unoriented_footprint);

  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      throw nav_core2::IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }

    // get the cell coord of the second point
    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      throw nav_core2::IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    throw nav_core2::IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }

  // get the cell coord of the first point
  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    throw nav_core2::IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }

  line_cost = lineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  // if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

double CollisionChecker::lineCost(int x0, int x1, int y0, int y1)
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}
double CollisionChecker::pointCost(int x, int y)
{
  unsigned char cost = costmap_->getCost(x, y);
  // if the cell is in an obstacle the path is invalid or unknown
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Unknown Region.");
  }

  return cost;
}

}  // namespace dwb_critics

