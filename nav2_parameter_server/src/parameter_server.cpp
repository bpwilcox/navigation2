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

#include "nav2_parameter_server/parameter_server.hpp"

#include <string>

#include "nav2_util/node_utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace nav2_parameter_server
{

ParameterServer::ParameterServer(const rclcpp::NodeOptions & /* options */)
: Node("parameter_server", nav2_util::get_node_options_default())
{
  RCLCPP_INFO(get_logger(), "Creating");
}

ParameterServer::~ParameterServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

}  // namespace nav2_parameter_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_parameter_server::ParameterServer)
