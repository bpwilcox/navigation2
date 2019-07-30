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

#include <memory>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/lifecycle_parameters_client.hpp"
#include "nav2_util/lifecycle_utils.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class ClientTestNode : public nav2_util::LifecycleNode
{
public:
  ClientTestNode(std::string name)
  : LifecycleNode(name, "", false)
  {
  }

  ~ClientTestNode() 
  {
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    parameter_client_ = std::make_unique<nav2_util::LifecycleParametersClient>(
      shared_from_this(), "robot_server");

    while (!parameter_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return nav2_util::CallbackReturn::FAILURE;
      }
    }

    robot_model_type_ = parameter_client_->get_parameter<std::string>(
      "robot_model_type", "diff");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    parameter_client_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  std::string get_robot_type()
  {
    return robot_model_type_;
  }

protected:
  std::unique_ptr<nav2_util::LifecycleParametersClient> parameter_client_;
  std::string robot_model_type_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    test_client_node_ = std::make_shared<ClientTestNode>("test_client_node");
    const unsigned number_of_threads = 3;
    const bool yield_thread_before_execute = false;

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::executor::ExecutorArgs(), number_of_threads, yield_thread_before_execute);

    thread_ = std::make_unique<std::thread>(
      [&]( rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) {
        executor_->add_node(node);
        executor_->spin();
        executor_->remove_node(node);
      },
      test_client_node_->get_node_base_interface());
  }

  ~TestNode()
  {
    executor_->cancel();
    thread_->join();
  }

protected:
  std::shared_ptr<ClientTestNode> test_client_node_;
  std::unique_ptr<std::thread> thread_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

TEST_F(TestNode, GetParameter)
{
  // test_client_node_->on_configure(test_client_node_->get_current_state());
  // std::vector<std::string>({"test_client_node"})
  nav2_util::bringup_lifecycle_nodes(std::vector<std::string>({"test_client_node"}));
  ASSERT_EQ(test_client_node_->get_robot_type(), std::string("differential"));
}

