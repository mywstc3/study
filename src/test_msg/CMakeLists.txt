cmake_minimum_required(VERSION 3.8)
project(test_msg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces - 必须在使用前调用
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(test_node src/test_node.cpp)

# Add dependencies
ament_target_dependencies(test_node
  "rclcpp"
  "std_msgs"
)

# Add include directories
target_include_directories(test_node PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 确保消息生成在节点编译之前 - 使用正确的目标名称
add_dependencies(test_node ${PROJECT_NAME}_generate_interfaces)

# Install targets
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install message files
install(
  FILES msg/Test.msg
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install generated headers
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()