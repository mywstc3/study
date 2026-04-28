#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{
constexpr int kNumMotors = 5;
}

// 节点类定义
class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode()
  : Node("sy5_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_rx", 10,
      std::bind(&MotorControlNode::serial_rx_callback, this, std::placeholders::_1));
    DuoJi_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "DuoJi_angle", 10,
      std::bind(&MotorControlNode::DuoJi_angle_callback, this, std::placeholders::_1));
    moto_target_angle_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "moto_target_angle", 10,
      std::bind(&MotorControlNode::moto_target_angle_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_tx", 10);
    DuoJi_publisher_ = this->create_publisher<std_msgs::msg::String>("DuoJi_serial_tx", 10);

    // 1. 声明每个电机的一组参数（5 个电机：motor0 ~ motor4）
    for (int i = 0; i < kNumMotors; i++) {
      std::string motor_prefix = "motor" + std::to_string(i) + ".";

      this->declare_parameter(motor_prefix + "task_flag", 3);  // 默认位置模式
      this->declare_parameter(motor_prefix + "moto_speed", 100);
      this->declare_parameter(motor_prefix + "moto_a_speed", 100);
      this->declare_parameter(motor_prefix + "moto_pose", 0);
      this->declare_parameter(motor_prefix + "moto_mod", 1);  // 默认绝对模式
      this->declare_parameter(motor_prefix + "moto_together_flag", 0);
      this->declare_parameter(motor_prefix + "moto_angle", 0.0);
    }
    this->declare_parameter<std::vector<double>>(
      "all_moto_angle_now", std::vector<double>(kNumMotors, 0.0));

    // 2. 默认值初始化
    motor_init();

    // 3. 把参数服务器里的初值同步进来
    update_parameters_from_server();

    // 4. 设置参数变更回调
    parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotorControlNode::parameters_callback, this, std::placeholders::_1));

    // 创建定时器，定期发送电机控制命令
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MotorControlNode::timer_callback, this));
    moto_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "all_moto_angle_now", 10);
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动");
  }

private:
  // ---------------- callbacks ----------------

  void DuoJi_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    DuoJi_contral(msg->data, 100);
    RCLCPP_DEBUG(this->get_logger(), "发送舵机角度指令到舵机 %.2f", msg->data);
  }

  void moto_target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < static_cast<size_t>(kNumMotors)) {
      RCLCPP_WARN(
        this->get_logger(),
        "moto_target_angle 数据长度 %zu < %d，已丢弃",
        msg->data.size(), kNumMotors);
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    std::array<double, kNumMotors> moto_angle_delta{};
    for (int i = 0; i < kNumMotors; i++) {
      moto_angle_[i] = msg->data[i];
      moto_angle_delta[i] = std::fabs(moto_angle_[i] - last_moto_angle_[i]);
    }
    const double max_angle = *std::max_element(
      moto_angle_delta.begin(), moto_angle_delta.end());

    // 当所有目标角与上次相同时 max_angle == 0，避免除零；统一退回到基础速度
    if (max_angle < 1e-6) {
      for (int i = 0; i < kNumMotors; i++) {
        moto_speed_[i] = moto_base_speed_;
      }
      return;
    }
    for (int i = 0; i < kNumMotors; i++) {
      moto_speed_[i] = static_cast<uint16_t>(
        moto_angle_delta[i] / max_angle * moto_base_speed_ + 0.5);
    }
  }

  void serial_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(
        this->get_logger(),
        "serial_rx 包过短(%zu)，已丢弃", msg->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    switch (msg->data[1]) {
      case 0x36:
        if (msg->data.size() == 8) {
          uint8_t motor_id = msg->data[0] - 1;
          if (motor_id >= kNumMotors) {
            RCLCPP_WARN(
              this->get_logger(),
              "角度数据包电机 ID 越界: %u", motor_id);
            break;
          }
          uint32_t angle_data = (static_cast<uint32_t>(msg->data[3]) << 24) |
            (static_cast<uint32_t>(msg->data[4]) << 16) |
            (static_cast<uint32_t>(msg->data[5]) << 8) |
            static_cast<uint32_t>(msg->data[6]);
          moto_angle_now_[motor_id] =
            static_cast<double>(angle_data) / 65536.0 / 50.0 * 360.0;
          // 角度跳变补偿（电机量化导致的回绕）
          if (moto_angle_[motor_id] > 93.6 && moto_angle_[motor_id] < 100.8) {
            if (moto_angle_now_[motor_id] > 72.0 &&
              moto_angle_now_[motor_id] < 79.2)
            {
              moto_angle_now_[motor_id] += 21.6;
            }
          }
          moto_angle_read_command_flag_[motor_id] = 1;
          channel_idle_flag_ = true;
        } else {
          RCLCPP_WARN(this->get_logger(), "接收到无效的角度数据包");
        }
        break;
      case 0xFD:
        if (msg->data.size() == 4) {
          uint8_t motor_id = msg->data[0] - 1;
          if (motor_id >= kNumMotors) {
            RCLCPP_WARN(
              this->get_logger(),
              "位置数据包电机 ID 越界: %u", motor_id);
            break;
          }
          if (msg->data[2] == 0x02) {
            RCLCPP_DEBUG(
              this->get_logger(),
              "电机 %u 运动命令发送成功", motor_id);
            moto_angle_sport_command_flag_[motor_id] = 1;
            channel_idle_flag_ = true;
          } else if (msg->data[2] == 0xE2) {
            RCLCPP_WARN(
              this->get_logger(),
              "电机 %u 运动命令发送失败", motor_id);
            moto_angle_sport_command_flag_[motor_id] = 0;
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "接收到无效的位置数据包");
        }
        break;
    }
  }

  // ---------------- parameter handling ----------------

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & param : parameters) {
      const std::string & param_name = param.get_name();

      size_t dot_pos = param_name.find('.');
      if (dot_pos == std::string::npos) {continue;}

      std::string motor_str = param_name.substr(0, dot_pos);
      std::string param_type = param_name.substr(dot_pos + 1);

      if (motor_str.compare(0, 5, "motor") != 0) {continue;}
      int motor_index = std::stoi(motor_str.substr(5));

      if (motor_index < 0 || motor_index >= kNumMotors) {
        RCLCPP_WARN(get_logger(), "无效的电机索引: %d", motor_index);
        continue;
      }

      if (param_type == "task_flag") {
        task_flag_[motor_index] = param.as_int();
        RCLCPP_INFO(
          get_logger(), "电机 %d task_flag 更新为: %d",
          motor_index, task_flag_[motor_index]);
      } else if (param_type == "moto_speed") {
        moto_speed_[motor_index] = static_cast<uint16_t>(param.as_int());
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_speed 更新为: %u",
          motor_index, moto_speed_[motor_index]);
      } else if (param_type == "moto_a_speed") {
        moto_a_speed_[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_a_speed 更新为: %u",
          motor_index, moto_a_speed_[motor_index]);
      } else if (param_type == "moto_pose") {
        moto_pose_[motor_index] = static_cast<uint32_t>(param.as_int());
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_pose 更新为: %u",
          motor_index, moto_pose_[motor_index]);
      } else if (param_type == "moto_mod") {
        moto_mod_[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_mod 更新为: %u",
          motor_index, moto_mod_[motor_index]);
      } else if (param_type == "moto_together_flag") {
        moto_together_flag_[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_together_flag 更新为: %u",
          motor_index, moto_together_flag_[motor_index]);
      } else if (param_type == "moto_angle") {
        moto_angle_[motor_index] = param.as_double();
        RCLCPP_INFO(
          get_logger(), "电机 %d moto_angle 更新为: %f",
          motor_index, moto_angle_[motor_index]);
      }
    }
    return result;
  }

  void update_parameters_from_server()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < kNumMotors; i++) {
      std::string motor_prefix = "motor" + std::to_string(i) + ".";

      task_flag_[i] = this->get_parameter(motor_prefix + "task_flag").as_int();
      moto_speed_[i] = static_cast<uint16_t>(
        this->get_parameter(motor_prefix + "moto_speed").as_int());
      moto_a_speed_[i] = static_cast<uint8_t>(
        this->get_parameter(motor_prefix + "moto_a_speed").as_int());
      moto_pose_[i] = static_cast<uint32_t>(
        this->get_parameter(motor_prefix + "moto_pose").as_int());
      moto_mod_[i] = static_cast<uint8_t>(
        this->get_parameter(motor_prefix + "moto_mod").as_int());
      moto_together_flag_[i] = static_cast<uint8_t>(
        this->get_parameter(motor_prefix + "moto_together_flag").as_int());
      moto_angle_[i] = this->get_parameter(motor_prefix + "moto_angle").as_double();
    }
  }

  // ---------------- low-level motor commands ----------------

  void moto_contral_location_mode(
    uint8_t motor_id, uint8_t direction,
    uint16_t speed, uint8_t a_speed, uint32_t pose,
    uint8_t mode, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(13);

    message.data[0] = motor_id;
    message.data[1] = 0xFD;
    message.data[2] = direction;
    message.data[3] = (speed >> 8) & 0xFF;
    message.data[4] = speed & 0xFF;
    message.data[5] = a_speed;
    message.data[6] = (pose >> 24) & 0xFF;
    message.data[7] = (pose >> 16) & 0xFF;
    message.data[8] = (pose >> 8) & 0xFF;
    message.data[9] = pose & 0xFF;
    message.data[10] = mode;
    message.data[11] = together_flag;
    message.data[12] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送位置模式指令到电机 %d", motor_id);
  }

  void moto_contral_speed_mode(
    uint8_t motor_id, uint8_t direction,
    uint16_t speed, uint8_t a_speed, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(8);

    message.data[0] = motor_id;
    message.data[1] = 0xF6;
    message.data[2] = direction;
    message.data[3] = (speed >> 8) & 0xFF;
    message.data[4] = speed & 0xFF;
    message.data[5] = a_speed;
    message.data[6] = together_flag;
    message.data[7] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送速度模式指令到电机 %d", motor_id);
  }

  void moto_contral_enable(uint8_t motor_id, uint8_t enable_flag, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(6);

    message.data[0] = motor_id;
    message.data[1] = 0xF3;
    message.data[2] = 0xAB;
    message.data[3] = enable_flag;
    message.data[4] = together_flag;
    message.data[5] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送使能指令到电机 %d", motor_id);
  }

  void moto_stop_now(uint8_t motor_id, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(5);

    message.data[0] = motor_id;
    message.data[1] = 0xFE;
    message.data[2] = 0xAB;
    message.data[3] = together_flag;
    message.data[4] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送急停指令到电机 %d", motor_id);
  }

  void moto_return_to_zero(uint8_t motor_id, uint8_t recover_mod, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(5);

    message.data[0] = motor_id;
    message.data[1] = 0x9A;
    message.data[2] = recover_mod;
    message.data[3] = together_flag;
    message.data[4] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送回零指令到电机 %d", motor_id);
  }

  void moto_move_together()
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(4);

    message.data[0] = 0x00;
    message.data[1] = 0xFD;
    message.data[2] = 0x02;
    message.data[3] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "同步运动开始");
  }

  void moto_angle_read(uint8_t motor_id)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(3);

    message.data[0] = motor_id;
    message.data[1] = 0x36;
    message.data[2] = 0x6B;

    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送角度读取指令到电机 %d", motor_id);
  }

  void DuoJi_contral(double angle, int time)
  {
    // 注意：必须用浮点运算，否则当 angle < 270 时 angle/270 整除会被截断为 0。
    int pwm = static_cast<int>(angle / 270.0 * 2000.0 + 500.0);
    std::stringstream ss;
    ss << "#000P" << pwm << 'T' << time << '!';

    auto message = std_msgs::msg::String();
    message.data = ss.str();

    RCLCPP_DEBUG(this->get_logger(), "舵机命令: %s", message.data.c_str());
    DuoJi_publisher_->publish(message);
  }

  // ---------------- bookkeeping ----------------

  void motor_init()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < kNumMotors; i++) {
      task_flag_[i] = 3;
      moto_a_speed_[i] = 100;
      moto_speed_[i] = 100;
      moto_pose_[i] = 0;
      last_moto_pose_[i] = 0;
      moto_mod_[i] = 1;
      moto_together_flag_[i] = 0;
      moto_angle_[i] = 0.0;
      moto_angle_now_[i] = 0.0;
      last_moto_angle_[i] = 0.0;
      moto_angle_sport_command_flag_[i] = 1;
      moto_angle_read_command_flag_[i] = 1;
    }
    channel_idle_flag_ = true;
    moto_i_ = 0;
  }

  bool check_moto_angle_sport_command_flag() const
  {
    for (int i = 0; i < kNumMotors; i++) {
      if (moto_angle_sport_command_flag_[i] != 1) {
        return false;
      }
    }
    return true;
  }

  bool check_moto_angle_read_command_flag() const
  {
    for (int i = 0; i < kNumMotors; i++) {
      if (moto_angle_read_command_flag_[i] != 1) {
        return false;
      }
    }
    return true;
  }

  void timer_callback()
  {
    std_msgs::msg::Float64MultiArray moto_angle_message;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (channel_idle_flag_ == false) {
        waiting_count_++;
        if (waiting_count_ < 20) {return;}
        // 等待超时，强制置回空闲
        channel_idle_flag_ = true;
      }
      waiting_count_ = 0;

      switch (task_flag_[moto_i_]) {
        case 0:  // 急停
          moto_stop_now(moto_i_ + 1, moto_together_flag_[moto_i_]);
          break;
        case 1:  // 监听模式
          moto_angle_read(moto_i_ + 1);
          break;
        case 2:  // 使能模式
          moto_contral_enable(moto_i_ + 1, 0x01, moto_together_flag_[moto_i_]);
          break;
        case 3:  // 位置模式
          if (channel_idle_flag_) {
            if (std::fabs(
                static_cast<double>(last_moto_pose_[moto_i_]) -
                static_cast<double>(moto_pose_[moto_i_])) > 0.001)
            {
              moto_angle_[moto_i_] =
                static_cast<uint32_t>(moto_pose_[moto_i_] / 3200 / 50 * 360);
              last_moto_pose_[moto_i_] = moto_pose_[moto_i_];
              last_moto_angle_[moto_i_] = moto_angle_[moto_i_];
              moto_contral_location_mode(
                moto_i_ + 1, moto_direction_[moto_i_], moto_speed_[moto_i_],
                moto_a_speed_[moto_i_], moto_pose_[moto_i_], moto_mod_[moto_i_],
                moto_together_flag_[moto_i_]);
              moto_angle_sport_command_flag_[moto_i_] = 0;
              channel_idle_flag_ = false;
            } else if (std::fabs(
                last_moto_angle_[moto_i_] -
                moto_angle_[moto_i_]) > 0.001)
            {
              moto_pose_[moto_i_] =
                static_cast<uint32_t>(moto_angle_[moto_i_] / 360 * 50 * 3200);
              last_moto_angle_[moto_i_] = moto_angle_[moto_i_];
              last_moto_pose_[moto_i_] = moto_pose_[moto_i_];
              moto_contral_location_mode(
                moto_i_ + 1, moto_direction_[moto_i_], moto_speed_[moto_i_],
                moto_a_speed_[moto_i_], moto_pose_[moto_i_], moto_mod_[moto_i_],
                moto_together_flag_[moto_i_]);
              moto_angle_sport_command_flag_[moto_i_] = 0;
              channel_idle_flag_ = false;
            } else if (std::fabs(
                moto_angle_now_[moto_i_] -
                moto_angle_[moto_i_]) > 0.001)
            {
              moto_angle_read(moto_i_ + 1);
              moto_angle_read_command_flag_[moto_i_] = 0;
              channel_idle_flag_ = false;
            }
          }
          break;
        case 4:  // 速度模式
          moto_contral_speed_mode(
            moto_i_ + 1, moto_direction_[moto_i_], moto_speed_[moto_i_],
            moto_a_speed_[moto_i_], moto_together_flag_[moto_i_]);
          break;
        case 5:  // 回零模式
          moto_return_to_zero(
            moto_i_ + 1, moto_direction_[moto_i_], moto_together_flag_[moto_i_]);
          break;
        default:
          RCLCPP_WARN(
            this->get_logger(),
            "未知任务标志: %d for motor %u",
            task_flag_[moto_i_], moto_i_);
          break;
      }
      moto_i_ = static_cast<uint8_t>((moto_i_ + 1) % kNumMotors);

      moto_angle_message.data.assign(
        moto_angle_now_.begin(), moto_angle_now_.end());

      std::stringstream ss;
      ss << "电机角度: ";
      for (int i = 0; i < kNumMotors; i++) {
        ss << i << ":" << std::fixed << std::setprecision(2)
           << moto_angle_now_[i] << "度 ";
      }
      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
    moto_angle_publisher_->publish(moto_angle_message);
  }

  // ---------------- state ----------------

  std::array<int, kNumMotors> task_flag_{};
  std::array<uint8_t, kNumMotors> moto_direction_{0x00, 0x01, 0x01, 0x00, 0x10};
  std::array<uint16_t, kNumMotors> moto_speed_{};
  std::array<uint8_t, kNumMotors> moto_a_speed_{};
  std::array<uint32_t, kNumMotors> moto_pose_{};
  std::array<uint32_t, kNumMotors> last_moto_pose_{};
  std::array<uint8_t, kNumMotors> moto_mod_{};
  std::array<uint8_t, kNumMotors> moto_together_flag_{};
  std::array<double, kNumMotors> moto_angle_{};
  std::array<double, kNumMotors> last_moto_angle_{};
  std::array<double, kNumMotors> moto_angle_now_{};
  std::array<int, kNumMotors> moto_angle_sport_command_flag_{};
  std::array<int, kNumMotors> moto_angle_read_command_flag_{};
  bool channel_idle_flag_ = true;
  uint8_t moto_i_ = 0;
  int waiting_count_ = 0;
  uint16_t moto_base_speed_ = 50;

  std::mutex state_mutex_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DuoJi_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr moto_target_angle_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr DuoJi_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr moto_angle_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorControlNode>();
  RCLCPP_INFO(node->get_logger(), "hello world sy_pkg package");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
