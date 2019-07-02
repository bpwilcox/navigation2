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

#ifndef NAV2_UTIL__NAV2_CLIENTS__CLEAR_AROUND_COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__NAV2_CLIENTS__CLEAR_AROUND_COSTMAP_SERVICE_CLIENT_HPP_

#include <string>
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"

namespace nav2_util
{

class ClearAroundCostmapServiceClient
  : public ServiceClient<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  explicit ClearAroundCostmapServiceClient(const std::string & service_name)
  : ServiceClient<nav2_msgs::srv::ClearCostmapAroundRobot>(service_name)
  {
  }

  using clearAroundCostmapServiceRequest =
    ServiceClient<nav2_msgs::srv::ClearCostmapAroundRobot>::RequestType;
  using clearAroundCostmapServiceResponse =
    ServiceClient<nav2_msgs::srv::ClearCostmapAroundRobot>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__NAV2_CLIENTS__CLEAR_AROUND_COSTMAP_SERVICE_CLIENT_HPP_
