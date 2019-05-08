#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{
    
class TestSubscriber
{
public:
  TestSubscriber(
    rclcpp::Node::SharedPtr & ros_node,
    std::string & costmap_topic,
    std::string & footprint_topic)
: footprint_sub_(ros_node, footprint_topic),
  costmap_sub_(ros_node, costmap_topic),
  node_(ros_node)
  {
    got_footprint_pub_ = node_->create_publisher<std_msgs::msg::Bool>("received_footprint");
    rclcpp::Rate loop_rate(1);

    while(rclcpp::ok())
    {
      RCLCPP_INFO(node_->get_logger(), "Received for footprint topic '%s':, %d", footprint_topic.c_str(), footprint_sub_.getFootprint(footprint_));
      std_msgs::msg::Bool is_received;
      is_received.data = footprint_sub_.getFootprint(footprint_);
      got_footprint_pub_->publish(is_received);
      Costmap2D * cm = costmap_sub_.getCostmap();
      bool costmap_received_;
      if (cm == nullptr) {
        costmap_received_ = false;
      } else {
        costmap_received_ = true;
      }
      RCLCPP_INFO(node_->get_logger(), "Received for costmap topic '%s':, %d", costmap_topic.c_str(), costmap_received_);

      rclcpp::spin_some(node_);
      loop_rate.sleep();
    }
  }
  ~TestSubscriber(){}
  
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr got_footprint_pub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  FootprintSubscriber footprint_sub_;
  CostmapSubscriber costmap_sub_;
};

}  // namespace nav2_costmap_2d

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
    nav2_costmap_2d::TestSubscriber MyTest(node, costmap_topic, footprint_topic);
  }
  catch(std::runtime_error& e)
  {
    RCLCPP_ERROR(node->get_logger(),"%s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}