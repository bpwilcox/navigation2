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

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_parameters_client.hpp"
#include "nav2_util/lifecycle_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = nav2_util::LifecycleNode::make_shared("robot_param_client");
  auto parameters_client = std::make_shared<nav2_util::LifecycleParametersClient>(node, "robot_server");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  std::stringstream ss;
  // Get a few of the parameters just set.
  for (auto & parameter : parameters_client->get_parameters({"robot_radius", "min_speed_xy"})) {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
  }

  auto radius = parameters_client->get_parameter<double>("robot_radius");
  std::string type = parameters_client->get_parameter<std::string>("robot_model_type");
  std::string name = parameters_client->get_parameter<std::string>("robot_name", "turtlebot");

  RCLCPP_INFO(node->get_logger(), "name: %s, type: %s, radius: %f",
    name.c_str(), type.c_str(), radius);

  RCLCPP_INFO(node->get_logger(), ss.str().c_str());
  
  rclcpp::shutdown();

  return 0;
}
