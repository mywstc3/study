#include "rclcpp/rclcpp.hpp"
#include "test_msg/msg/test.hpp"

using test_msg::msg::Test;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // 创建一个自定义消息实例
  auto msg = std::make_shared<Test>();
  msg->data = {1, 2, 3};
  msg->sizes = {4, 4, 4};
  
  RCLCPP_INFO(node->get_logger(), "Test message created successfully!");
  RCLCPP_INFO(node->get_logger(), "Data: %zu, %zu, %zu", msg->data[0], msg->data[1], msg->data[2]);
  RCLCPP_INFO(node->get_logger(), "Sizes: %zu, %zu, %zu", msg->sizes[0], msg->sizes[1], msg->sizes[2]);
  
  rclcpp::shutdown();
  return 0;
}