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
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

class ParametersClient : public rclcpp::SyncParametersClient
{
public:
  ParametersClient(
    const std::string & remote_node_name = "")
  : rclcpp::SyncParametersClient(
      generate_internal_node(remote_node_name + "_param_client_Node"),
      remote_node_name)
  {}
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__PARAMETERS_CLIENT_HPP_
