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

namespace nav2_util
{
template<typename NodeT>
class LifecycleNodeMonitor
{
public:
  LifecycleNodeMonitor(
    NodeT node,
    const std::vector<std::string> node_names);

protected:

};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_MONITOR_HPP_