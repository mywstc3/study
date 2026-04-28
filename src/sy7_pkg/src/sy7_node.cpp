#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr int kNumAxles = 6;

struct Axle
{
  double axle_long = 0.0;
  double projection_x = 0.0;
  double projection_y = 0.0;
  double projection_z = 0.0;
  double angle_a = 0.0;
  double angle_last = 0.0;
};
}  // namespace

class Location_Resolve_Node : public rclcpp::Node
{
public:
  Location_Resolve_Node()
  : Node("sy7_node")
  {
    // 第 4 关节（axle index 3）目前由几何法约束为常量；这里暴露成参数，
    // 用户后续可以用 `ros2 param set /sy7_node axle3_target_angle <deg>` 动态调整，
    // 也方便在 launch 里覆盖。完整 6-DOF 反解需要末端姿态输入，那是后续工作。
    this->declare_parameter<double>("axle3_target_angle", 0.0);

    location_target_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "location_target", 10,
      std::bind(&Location_Resolve_Node::location_target_callback, this, std::placeholders::_1));

    moto_angle_target_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "moto_target_angle", 10);

    RCLCPP_INFO(this->get_logger(), "逆解算节点已启动");
  }

  ~Location_Resolve_Node() override
  {
    RCLCPP_INFO(this->get_logger(), "逆解算节点已关闭");
  }

private:
  void location_target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "location_target 数据长度 %zu < 3，已丢弃", msg->data.size());
      return;
    }
    const double target_x = msg->data[0];
    const double target_y = msg->data[1];
    const double target_z = msg->data[2];

    std_msgs::msg::Float64MultiArray message;
    message.data.resize(kNumAxles);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      location_target_angle_solve(target_x, target_y, target_z);
      for (int i = 0; i < kNumAxles; i++) {
        message.data[i] = axles_[i].angle_a;
      }
    }
    moto_angle_target_publisher_->publish(message);
  }

  bool check_angle() const
  {
    for (int i = 0; i < 5; i++) {
      if (axles_[i].angle_a > 180 || axles_[i].angle_a < 0 ||
        std::isnan(axles_[i].angle_a))
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "电机 %d 角度超出范围: %.2f", i, axles_[i].angle_a);
        return false;
      }
    }
    return true;
  }

  void location_target_angle_solve(double target_x, double target_y, double target_z)
  {
    axles_[0].axle_long = 130.5;
    axles_[1].axle_long = 40.0;
    axles_[2].axle_long = 180.0;
    axles_[3].axle_long = 168.0;
    axles_[4].axle_long = 127.5;
    axles_[5].axle_long = 143.0;

    const double target_projection_x = std::sqrt(target_x * target_x + target_y * target_y);

    axles_[0].projection_x = 0;
    axles_[0].projection_y = 0;
    axles_[0].projection_z = axles_[0].axle_long;

    axles_[1].projection_x = 0;
    axles_[1].projection_y = 0;
    axles_[1].projection_z = axles_[0].axle_long + axles_[1].axle_long;

    axles_[5].projection_x = target_projection_x;
    axles_[5].projection_y = 0;
    axles_[5].projection_z = target_z;

    axles_[4].projection_x = axles_[5].projection_x;
    axles_[4].projection_y = 0;
    axles_[4].projection_z = axles_[5].projection_z + axles_[5].axle_long;

    axles_[0].angle_a = (PI - std::atan2(target_y, target_x)) / PI * 180;

    const double axle_24_long = std::sqrt(
      axles_[3].axle_long * axles_[3].axle_long +
      axles_[4].axle_long * axles_[4].axle_long);
    const double axle_14_long = std::sqrt(
      std::abs(axles_[1].projection_z - axles_[4].projection_z) *
      std::abs(axles_[1].projection_z - axles_[4].projection_z) +
      axles_[4].projection_x * axles_[4].projection_x);
    const double axle_15_long = std::sqrt(
      std::abs(axles_[1].projection_z - axles_[5].projection_z) *
      std::abs(axles_[1].projection_z - axles_[5].projection_z) +
      axles_[5].projection_x * axles_[5].projection_x);
    const double angle_41p = std::atan2(
      axles_[4].projection_z - axles_[1].projection_z, axles_[4].projection_x);
    const double angle_214 = std::acos(
      (axle_14_long * axle_14_long + axles_[2].axle_long * axles_[2].axle_long -
      axle_24_long * axle_24_long) /
      (2 * axle_14_long * axles_[2].axle_long));

    // 第 4 关节角度（轴 3）：取 ROS 参数，默认 0。
    axles_[3].angle_a = this->get_parameter("axle3_target_angle").as_double();
    axles_[1].angle_a = 180 - (angle_41p + angle_214) / PI * 180;

    const double angle_124 = std::acos(
      (axles_[2].axle_long * axles_[2].axle_long + axle_24_long * axle_24_long -
      axle_14_long * axle_14_long) /
      (2 * axles_[2].axle_long * axle_24_long));
    const double angle_423 = std::atan2(axles_[4].axle_long, axles_[3].axle_long);
    axles_[2].angle_a = (angle_124 + angle_423) / PI * 180 - 90;

    const double angle_243 = std::atan2(axles_[3].axle_long, axles_[4].axle_long);
    const double angle_142 = std::acos(
      (axle_14_long * axle_14_long + axle_24_long * axle_24_long -
      axles_[2].axle_long * axles_[2].axle_long) /
      (2 * axle_14_long * axle_24_long));
    const double angle_145 = std::acos(
      (axle_14_long * axle_14_long + axles_[5].axle_long * axles_[5].axle_long -
      axle_15_long * axle_15_long) /
      (2 * axle_14_long * axles_[5].axle_long));
    axles_[4].angle_a = 270 - (angle_243 + angle_142 + angle_145) / PI * 180;

    if (!check_angle()) {
      RCLCPP_ERROR(this->get_logger(), "位置不可达，回退到上一次有效角度");
      for (int j = 0; j < 5; j++) {
        axles_[j].angle_a = axles_[j].angle_last;
      }
    } else {
      for (int j = 0; j < 5; j++) {
        axles_[j].angle_last = axles_[j].angle_a;
      }
    }

    std::stringstream ss;
    ss << "电机角度: ";
    for (int i = 0; i < kNumAxles; i++) {
      ss << i << ":" << std::fixed << std::setprecision(2) << axles_[i].angle_a << "度 ";
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
  }

  std::array<Axle, kNumAxles> axles_{};
  std::mutex state_mutex_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr location_target_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr moto_angle_target_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Location_Resolve_Node>());
  rclcpp::shutdown();
  return 0;
}
