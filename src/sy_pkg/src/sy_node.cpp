#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sy_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic",10);
  std_msgs::msg::String message;
  message.data = "Hello Ros2";
  rclcpp::Rate rate(1);
  printf("hello world sy_pkg package\n");
  while (rclcpp::ok())
  {
    publisher->publish(message);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
