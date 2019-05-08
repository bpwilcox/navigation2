#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/collision_checker.hpp"
#include "std_msgs/msg/bool.hpp"

class TestCollisionChecker : public dwb_critics::CollisionChecker
{
public:
  TestCollisionChecker(
    rclcpp::Node::SharedPtr & ros_node,
    std::string costmap_topic,
    std::string footprint_topic)
: CollisionChecker(ros_node, costmap_topic, footprint_topic),
  node_(ros_node)
  {
    got_costmap_pub_ = node_->create_publisher<std_msgs::msg::Bool>("received_footprint");
    rclcpp::Rate loop_rate(1);

    while(rclcpp::ok())
    {
      RCLCPP_INFO(node_->get_logger(), "Listening for footprint topic: %s", footprint_topic_.c_str());
      std_msgs::msg::Bool is_received;
      is_received.data = footprint_sub_->getFootprint(footprint_);
      got_costmap_pub_->publish(is_received);
      loop_rate.sleep();
    }
  }
  ~TestCollisionChecker(){}
  
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr got_costmap_pub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // This node is created and spun in order to pass clock to buffer for Costmap2DROS node
  auto node = rclcpp::Node::make_shared("test_node");

  std::string costmap_topic;
  std::string footprint_topic;
  node->declare_parameter("costmap_topic");
  node->declare_parameter("footprint_topic");
  node->get_parameter_or<std::string>("costmap_topic", costmap_topic, "local_costmap/costmap_raw");
  node->get_parameter_or<std::string>("footprint_topic", footprint_topic, "local_costmap/footprint"); 

  try
  {
    TestCollisionChecker MyTest(node, costmap_topic, footprint_topic);
  }
  catch(std::runtime_error& e)
  {
    RCLCPP_ERROR(node->get_logger(),"%s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}