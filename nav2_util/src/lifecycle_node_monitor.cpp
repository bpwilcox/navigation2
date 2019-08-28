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

#include "nav2_util/lifecycle_node_monitor.hpp"

using lifecycle_msgs::msg::State;

namespace nav2_util
{

LifecycleNodeMonitor::LifecycleNodeMonitor(
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
const std::vector<std::string> node_names,
const rclcpp::QoS & qos)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  node_names_(node_names),
  qos_(qos)
{
  for (auto & name : node_names_) {
    node_state_map_[name] = State::PRIMARY_STATE_UNKNOWN;
    std::string topic = name + "/transition_event";
    RCLCPP_INFO(node_logging_->get_logger(), "Subscribing to topic: %s", topic.c_str());

    auto cb = [&name, this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr event) {
      node_state_map_[name] = event->goal_state.id;
      auto label = event->goal_state.label;
      RCLCPP_INFO(node_logging_->get_logger(), "Node '%s', state: %s", name.c_str(), label.c_str());
    };
    event_callbacks_.push_back(cb);

    auto sub = rclcpp::create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      node_topics_, topic, qos_, cb);
    event_subscriptions_.push_back(sub);
  }
}

std::uint8_t LifecycleNodeMonitor::get_status(std::string node_name)
{
  return node_state_map_[node_name];
}

}  // namespace nav2_util
