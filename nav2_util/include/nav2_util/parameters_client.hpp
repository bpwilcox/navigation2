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

#ifndef NAV2_UTIL__PARAMETERS_CLIENT_HPP_
#define NAV2_UTIL__PARAMETERS_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_util
{

class ParametersClient : public rclcpp::AsyncParametersClient
{
public:
  ParametersClient(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
  : rclcpp::AsyncParametersClient(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      remote_node_name,
      qos_profile,
      node->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant)),
    node_base_(node->get_node_base_interface())
  {}

  ParametersClient(
    rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
  : rclcpp::AsyncParametersClient(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      remote_node_name,
      qos_profile,
      node->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant)),
    node_base_(node->get_node_base_interface())
  {}

  template<typename T>
  T
  get_parameter_impl(
    const std::string & parameter_name, std::function<T()> parameter_not_found_handler)
  {
    std::vector<std::string> names;
    names.push_back(parameter_name);
    auto f = get_parameters(names);

    if (!nodeIsSpinable()) {
      auto status = f.wait_for(std::chrono::seconds(1));
      if (status != std::future_status::ready) {
        return parameter_not_found_handler();
      }
    } else {
      if (rclcpp::spin_until_future_complete(node_base_, f, std::chrono::seconds::max()) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        return parameter_not_found_handler();
      }
    }

    auto vars = f.get();

    if ((vars.size() != 1) || (vars[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)) {
      RCLCPP_WARN(rclcpp::get_logger("Parameter Client"), "Parameter '%s' not set",
       parameter_name.c_str());
      return parameter_not_found_handler();
    } else {
      return static_cast<T>(vars[0].get_value<T>());
    }
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name, const T & default_value)
  {
    return get_parameter_impl(
      parameter_name,
      std::function<T()>([&default_value]() -> T {return default_value;}));
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name)
  {
    return get_parameter_impl(
      parameter_name,
      std::function<T()>([parameter_name]() -> T {throw std::runtime_error("Parameter '" + parameter_name + "' is not set");}));
  }

  bool nodeIsSpinable() const
  {
    // Nodes can only be added to one executor. If it hasn't, then it can be spinned
    return !static_cast<bool>(node_base_->get_associated_with_executor_atomic());
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__PARAMETERS_CLIENT_HPP_
