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

#ifndef NAV2_UTIL__LIFECYCLE_NODE_MONITOR_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_MONITOR_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

namespace nav2_util
{
class LifecycleNodeMonitor
{
public:
  LifecycleNodeMonitor(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::vector<std::string> node_names,
    const rclcpp::QoS & qos =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));

  template<typename NodeT>
  LifecycleNodeMonitor(
    NodeT node,
    const std::vector<std::string> node_names,
    const rclcpp::QoS & qos =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)))
  : LifecycleNodeMonitor(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node_names,
    qos)
  {}

  std::uint8_t get_status(std::string node_name);

protected:
  // Node Interfaces used for logging and creating subscribers
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  std::vector<std::string> node_names_;

  rclcpp::QoS qos_;

  std::vector<rclcpp::Subscription
    <lifecycle_msgs::msg::TransitionEvent>::SharedPtr> event_subscriptions_;

  std::vector<std::function<void
    (const lifecycle_msgs::msg::TransitionEvent::SharedPtr &)>> event_callbacks_;

  std::map<std::string, std::uint8_t> node_state_map_;

};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_MONITOR_HPP_
