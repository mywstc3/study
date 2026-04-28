// Copyright 2025 mywstc3
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

#include "rclcpp/rclcpp.hpp"
#include "test_msg/msg/test.hpp"

using test_msg::msg::Test;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto msg = std::make_shared<Test>();
  msg->data = {1, 2, 3};
  msg->sizes = {4, 4, 4};

  RCLCPP_INFO(node->get_logger(), "Test message created successfully!");
  RCLCPP_INFO(
    node->get_logger(), "Data: %u, %u, %u",
    msg->data[0], msg->data[1], msg->data[2]);
  RCLCPP_INFO(
    node->get_logger(), "Sizes: %u, %u, %u",
    static_cast<unsigned>(msg->sizes[0]),
    static_cast<unsigned>(msg->sizes[1]),
    static_cast<unsigned>(msg->sizes[2]));

  rclcpp::shutdown();
  return 0;
}
