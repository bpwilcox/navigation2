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

#ifndef NAV2_UTIL__LIFECYCLE_PARAMETERS_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_PARAMETERS_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_util
{

class LifecycleParametersClient : public rclcpp::SyncParametersClient
{
public:
  LifecycleParametersClient(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
  : rclcpp::SyncParametersClient(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      remote_node_name,
      qos_profile)
  {}

  LifecycleParametersClient(
    rclcpp::executor::Executor::SharedPtr executor,
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
  : rclcpp::SyncParametersClient(
      executor,
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      remote_node_name,
      qos_profile)
  {}
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_PARAMETERS_CLIENT_HPP_
