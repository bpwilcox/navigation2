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
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/param_event_subscriber.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node1_ = std::make_shared<rclcpp::Node>(
      "test_param_event_subscriber", nav2_util::get_node_options_default());
    node2_ = std::make_shared<rclcpp::Node>(
      "test_param_event_subscriber", "test", nav2_util::get_node_options_default());

    ParamSubscriber = std::make_shared<nav2_util::ParamEventSubscriber>(
      node1_->get_node_base_interface(),
      node1_->get_node_topics_interface(),
      node1_->get_node_logging_interface()
    );

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    thread_ = std::make_unique<std::thread>(
      [&]() {
        executor_->add_node(node1_);
        executor_->add_node(node2_);

        executor_->spin();
        executor_->remove_node(node1_);
        executor_->remove_node(node2_);

      });
  }

  ~TestNode()
  {
    executor_->cancel();
    thread_->join();
  }

protected:
  rclcpp::Node::SharedPtr node1_;
  rclcpp::Node::SharedPtr node2_;
  std::unique_ptr<std::thread> thread_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<nav2_util::ParamEventSubscriber> ParamSubscriber;
};

TEST_F(TestNode, RegisterParameterUpdates)
{
  double double_param;
  bool bool_param;
  rclcpp::Rate r(100ms);  
  
  // Set individual parameters
  ParamSubscriber->register_param_update("my_double", double_param);
  ParamSubscriber->register_param_update("my_bool", bool_param, node2_->get_fully_qualified_name());

  node1_->set_parameter(rclcpp::Parameter("my_double", 1.0));
  r.sleep();

  node2_->set_parameter(rclcpp::Parameter("my_bool", true));
  r.sleep();

  ASSERT_EQ(double_param, 1.0);
  ASSERT_EQ(bool_param, true);

  bool received = false;
  ParamSubscriber->register_param_callback("my_string", [&received](){received = true;});
  node1_->set_parameter(rclcpp::Parameter("my_string", "test"));
  r.sleep();

  ASSERT_EQ(received, true);

  received = false;
  ParamSubscriber->register_param_callback(
    "my_int", [&received](){received = true;}, node2_->get_fully_qualified_name());
  node2_->set_parameter(rclcpp::Parameter("my_int", 1));
  r.sleep();
  ASSERT_EQ(received, true);

  // Set multiple parameters atomically
  received = false;
  node1_->set_parameters_atomically({
    rclcpp::Parameter("my_double", 2.0),
    rclcpp::Parameter("my_string", "test2")
    });
  r.sleep();
  
  ASSERT_EQ(double_param, 2.0);
  ASSERT_EQ(received, true);

  received = false;
  node2_->set_parameters_atomically({
    rclcpp::Parameter("my_bool", false),
    rclcpp::Parameter("my_int", 2)
    });
  r.sleep();
  
  ASSERT_EQ(bool_param, false);
  ASSERT_EQ(received, true);

  // Add User Callback
  double product;
  int int_param = 0;

  auto cb  = 
    [&int_param, &double_param, &product, this](const rcl_interfaces::msg::ParameterEvent::SharedPtr &)
    {
      ParamSubscriber->get_param_update("my_int", int_param);
      product = int_param * double_param;
      RCLCPP_INFO(node1_->get_logger(), "%f x %u = %f", double_param, int_param, product);
    };
  ParamSubscriber->set_event_callback(cb);
  
  node2_->set_parameter(rclcpp::Parameter("my_int", 3));
  node1_->set_parameter(rclcpp::Parameter("my_double", 5.0));
  r.sleep();
  ASSERT_EQ(product, 15.0);
}
