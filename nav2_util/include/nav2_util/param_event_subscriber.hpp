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

#ifndef NAV2_UTIL__PARAM_EVENT_SUBSCRIBER_HPP_
#define NAV2_UTIL__PARAM_EVENT_SUBSCRIBER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_util/string_utils.hpp"

namespace nav2_util
{

class ParamEventSubscriber
{
public:
  ParamEventSubscriber(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const rclcpp::QoS & qos =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)));

  // Sets user callback as a member variable.
  void set_event_callback(
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
    const std::string & node_namespace);

  template <typename ParameterT>
  void register_param_update(
    const std::string & parameter_name, ParameterT & parameter, const std::string & node_name = "")
  {

    auto full_node_name = resolve_node_path(node_name);

    add_namespace_event_subscriber(split_path(full_node_name).first);

    param_node_map_[parameter_name] = full_node_name;

    param_callbacks_[parameter_name] = 
      [&parameter, this](const std::string & parameter_name) {
        get_param_update<ParameterT>(parameter_name, parameter);
      };
  }

protected:

  // Adds a subscription to a namespace parameter events topic
  void add_namespace_event_subscriber(const std::string & node_namespace);

  void event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  std::string resolve_node_path(const std::string & path);
  
  template <typename ParameterT>
  void get_param_update(const std::string & parameter_name, ParameterT & value)
  {
    rclcpp::ParameterEventsFilter filter(last_event_, {parameter_name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    if (!filter.get_events().empty()) {
      RCLCPP_INFO(node_logging_->get_logger(), "Updating parameter: %s", parameter_name.c_str());
      auto param_msg = filter.get_events()[0].second;
      auto param = rclcpp::Parameter::from_parameter_msg(*param_msg);
      value = param.get_value<ParameterT>();
    }
  }

  // Interfaces used for logging and creating subscribers  
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  rclcpp::QoS qos_;

  std::map<std::string, std::string> param_node_map_;
  std::map<std::string, std::function<void(const std::string &)>> param_callbacks_;

  // Vector of unique namespaces added
  std::vector<std::string> node_namespaces_;

  // vector of event subscriptions for each namespace
  std::vector<rclcpp::Subscription
    <rcl_interfaces::msg::ParameterEvent>::SharedPtr> event_subscriptions_;

  // Users of this class will pass in an event callback
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> user_callback_;

  // Pointer to latest event message
  rcl_interfaces::msg::ParameterEvent::SharedPtr last_event_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__PARAM_EVENT_SUBSCRIBER_HPP_
