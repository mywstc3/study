#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void message_callback(const std_msgs::msg::String::SharedPtr msg)
{
  printf("收到消息: %s\n", msg->data.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<rclcpp::Node>("sy2_node");
  auto subscription = node->create_subscription<std_msgs::msg::String>("topic", 10, message_callback);
  printf("hello world sy2_pkg package\n");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
