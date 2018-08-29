#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <memory>

// rclcpp::init can only be called once per process, so this needs to be a global variable
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
    node = rclcpp::Node::make_shared("dwa_controller_test");
    client = std::make_unique<FollowPathTaskClient>("DwaController", node.get());
    while (node->count_subscribers("/DwaController_command") < 1) {
      rclcpp::spin_some(node);
    }
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<FollowPathTaskClient> client;
};

TEST_F(TestNode, ResultReturned)
{
  FollowPathCommand c;
  client->executeAsync(std::make_shared<FollowPathCommand>(c));
  FollowPathResult r;
  auto r_ptr = std::make_shared<FollowPathResult>(r);
  while (client->waitForResult(r_ptr, 1000) == TaskStatus::RUNNING) {
    rclcpp::spin_some(node);
  }
  SUCCEED();
}