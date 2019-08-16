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

#include "nav2_util/param_event_subscriber.hpp"
#include "nav2_util/string_utils.hpp"

namespace nav2_util
{

ParamEventSubscriber::ParamEventSubscriber(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const rclcpp::QoS & qos)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  qos_(qos)
{}

// Adds a subscription to a namespace parameter events topic
void ParamEventSubscriber::add_namespace_event_subscriber(const std::string & node_namespace)
{
  if (std::find(node_namespaces_.begin(), node_namespaces_.end(),
    node_namespace) == node_namespaces_.end())
  {
    node_namespaces_.push_back(node_namespace);
    auto topic = join_path(node_namespace, "parameter_events");
    RCLCPP_INFO(node_logging_->get_logger(), "Subscribing to topic: %s", topic.c_str());

    auto event_sub = rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(node_topics_,
      topic, qos_, std::bind(&ParamEventSubscriber::event_callback, this, std::placeholders::_1));
    event_subscriptions_.push_back(event_sub);
  }
}

void ParamEventSubscriber::set_event_callback(
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
  const std::string & node_namespace)
{
  auto full_namespace = resolve_node_path(node_namespace);
  add_namespace_event_subscriber(full_namespace);  
  user_callback_ = callback;
}

void ParamEventSubscriber::event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::string node_name = event->node;
  RCLCPP_INFO(node_logging_->get_logger(), "Parameter event received for node: %s", node_name.c_str());

  last_event_ = event;

  for (std::map<std::string, std::string>::iterator it = param_node_map_.begin();
    it != param_node_map_.end(); ++it)
  {
    if (node_name == it->second) {
      param_callbacks_[it->first](it->first);
    }
  }

  if (user_callback_) {
    user_callback_(event);
  }
}

std::string ParamEventSubscriber::resolve_node_path(const std::string & node_name)
{
  std::string full_path;
  if (node_name == "") {
    full_path = node_base_->get_fully_qualified_name();
  } else {
    full_path = node_name;
    if (*full_path.begin() != '/') {
      full_path = '/' + full_path;
    }
  }

  return full_path;
}

}  // namespace nav2_util
