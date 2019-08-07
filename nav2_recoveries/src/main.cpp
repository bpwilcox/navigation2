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
// limitations under the License. Reserved.

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_recoveries/back_up.hpp"
#include "nav2_recoveries/spin.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto recoveries_node = rclcpp::Node::make_shared("recoveries");

  recoveries_node->declare_parameter(
    "costmap_topic", rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
  recoveries_node->declare_parameter(
    "footprint_topic", rclcpp::ParameterValue(std::string("global_costmap/published_footprint")));

  auto spin = std::make_shared<nav2_recoveries::Spin>(
    recoveries_node);

  auto back_up = std::make_shared<nav2_recoveries::BackUp>(
    recoveries_node);

  const unsigned number_of_threads = 2;
  const bool yield_thread_before_execute = false;

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::ExecutorArgs(), number_of_threads, yield_thread_before_execute);

  executor.add_node(recoveries_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
