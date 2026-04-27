#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// 全局变量
int task_flag[6]; // 0急停 1监听模式 2使能模式 3位置模式 4速度模式 5回零模式
uint8_t moto_derection[5] = {0x00, 0x01, 0x01, 0x00, 0x10};
uint16_t moto_speed[5];
uint8_t moto_a_speed[5];
uint32_t moto_pose[5];
uint32_t moto_pose_now[5];
uint32_t last_moto_pose[5];
uint8_t moto_mod[5];
uint8_t moto_together_flag[5];
double moto_angle[5];
double last_moto_angle[5];
double moto_angle_now[5];
int moto_angle_sport_command_flag[6];
int moto_angle_read_command_flag[6]; 
bool channel_idle_flag = true; // 通道空闲标志
uint8_t moto_i=0x00;
int waiting_count=0;
uint16_t moto_base_speed = 50;
// 节点类定义
class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode() : Node("sy5_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_rx", 10, std::bind(&MotorControlNode::serial_rx_callback, this, std::placeholders::_1));
    DuoJi_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "DuoJi_angle", 10, std::bind(&MotorControlNode::DuoJi_angle_callback, this, std::placeholders::_1));
    moto_target_angle_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "moto_target_angle", 10, std::bind(&MotorControlNode::moto_target_angle_callback, this, std::placeholders::_1));
    // 创建发布者，发布UInt8MultiArray消息到serial_tx话题
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_tx", 10);
    DuoJi_publisher_ = this->create_publisher<std_msgs::msg::String>("DuoJi_serial_tx", 10);
    // 1. 声明参数并为每个电机声明一组参数
    for (int i = 0; i < 6; i++) {
      std::string motor_prefix = "motor" + std::to_string(i) + ".";

      this->declare_parameter(motor_prefix + "task_flag", 3); // 默认位置模式
      this->declare_parameter(motor_prefix + "moto_speed", 100);
      this->declare_parameter(motor_prefix + "moto_a_speed", 100);
      this->declare_parameter(motor_prefix + "moto_pose", 0);
      this->declare_parameter(motor_prefix + "moto_mod", 1);// 默认绝对模式
      this->declare_parameter(motor_prefix + "moto_together_flag", 0);
      this->declare_parameter(motor_prefix + "moto_angle", 0.0);
    }
    this->declare_parameter<std::vector<double>>("all_moto_angle_now", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    // 2. 获取参数的初始值并初始化全局数组
    update_parameters_from_server();

    // 3. 设置参数变更回调
    parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MotorControlNode::parameters_callback, this, std::placeholders::_1));

    // 初始化电机参数
    motor_init();
    
    // 创建定时器，定期发送电机控制命令
    timer_ = this->create_wall_timer(100ms, std::bind(&MotorControlNode::timer_callback, this));
    moto_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("all_moto_angle_now", 10);
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动");
  }

private:
  void DuoJi_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float angle = msg->data;
    DuoJi_contral(angle,100);
    RCLCPP_DEBUG(this->get_logger(), "发送舵机角度指令到舵机 %.2f", msg->data);

  }
  void moto_target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 5) {
      RCLCPP_WARN(this->get_logger(),
                  "moto_target_angle 数据长度 %zu < 5，已丢弃", msg->data.size());
      return;
    }
    double moto_angle_c[5];
    for(int i=0;i<5;i++){
      moto_angle[i] = msg->data[i];
      moto_angle_c[i] = std::fabs(moto_angle[i]-last_moto_angle[i]);
    }
    auto max_angle_iter = std::max_element(moto_angle_c, moto_angle_c + 5);
    double max_angle = *max_angle_iter;
    // 当所有目标角与上次相同时 max_angle == 0，避免除零；统一退回到基础速度
    if (max_angle < 1e-6) {
      for(int i=0;i<5;i++){
        moto_speed[i] = moto_base_speed;
      }
      return;
    }
    for(int i=0;i<5;i++){
      moto_speed[i] = static_cast<uint16_t>(moto_angle_c[i] / max_angle * moto_base_speed + 0.5);
    }
  }

  void serial_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(),
                  "serial_rx 包过短(%zu)，已丢弃", msg->data.size());
      return;
    }
    switch (msg->data[1])
    {
      case 0x36:
        if (msg->data.size() == 8) {
          uint8_t motor_id = msg->data[0]-1;
          uint32_t angle_data = (static_cast<uint32_t>(msg->data[3]) << 24) |
                                (static_cast<uint32_t>(msg->data[4]) << 16) |
                                (static_cast<uint32_t>(msg->data[5]) << 8) |
                                static_cast<uint32_t>(msg->data[6]);
          moto_angle_now[motor_id] = static_cast<double>(angle_data)/65536/50*360;
          //  RCLCPP_INFO(this->get_logger(), "电机 %d 角度:%.2f", motor_id,moto_angle_now[motor_id]);
          if(moto_angle[motor_id]>93.6&&moto_angle[motor_id]<100.8)
          {
            if(moto_angle_now[motor_id]>72.0&&moto_angle_now[motor_id]<79.2){
              moto_angle_now[motor_id] += 21.6;
              // RCLCPP_WARN(this->get_logger(), "接收到无效的角度数据包");
            }
          }
          moto_angle_read_command_flag[motor_id] = 1;
          channel_idle_flag = true;
        } else {
          RCLCPP_WARN(this->get_logger(), "接收到无效的角度数据包");
        }
         break;
      case 0xFD: 
        if (msg->data.size() == 4) {
          uint8_t motor_id = msg->data[0]-1;
          if(msg->data[2]==0x02){
            RCLCPP_DEBUG(this->get_logger(), "电机 %d 运动命令发送成功", motor_id);
            moto_angle_sport_command_flag[motor_id] = 1;
            channel_idle_flag = true;

          }
          else if(msg->data[2]==0xE2){
            RCLCPP_WARN(this->get_logger(), "电机 %d 运动命令发送失败", motor_id);
            moto_angle_sport_command_flag[motor_id] = 0;
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "接收到无效的位置数据包");
        }
        break;
    }
  }
  // 4. 定义参数回调函数
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      std::string param_name = param.get_name();
      
      // 解析参数名，例如 "motor0.task_flag"
      size_t dot_pos = param_name.find('.');
      if (dot_pos == std::string::npos) continue;

      std::string motor_str = param_name.substr(0, dot_pos); // 例如 "motor0"
      std::string param_type = param_name.substr(dot_pos + 1); // 例如 "task_flag"

      // 提取电机索引
      if (motor_str.compare(0, 5, "motor") != 0) continue;
      int motor_index = std::stoi(motor_str.substr(5));

      if (motor_index < 0 || motor_index >= 5) {
        RCLCPP_WARN(get_logger(), "无效的电机索引: %d", motor_index);
        continue;
      }

      // 根据参数类型更新对应的全局数组
      if (param_type == "task_flag") {
        task_flag[motor_index] = param.as_int();
        RCLCPP_INFO(get_logger(), "电机 %d task_flag 更新为: %d", motor_index, task_flag[motor_index]);
      } else if (param_type == "moto_speed") {
        moto_speed[motor_index] = static_cast<uint16_t>(param.as_int());
        RCLCPP_INFO(get_logger(), "电机 %d moto_speed 更新为: %u", motor_index, moto_speed[motor_index]);
      } else if (param_type == "moto_a_speed") {
        moto_a_speed[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(get_logger(), "电机 %d moto_a_speed 更新为: %u", motor_index, moto_a_speed[motor_index]);
      }
      else if (param_type == "moto_pose") {
        moto_pose[motor_index] = static_cast<uint32_t>(param.as_int());
        RCLCPP_INFO(get_logger(), "电机 %d moto_pose 更新为: %u", motor_index, moto_pose[motor_index]);
      }
      else if (param_type == "moto_mod") {
        moto_mod[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(get_logger(), "电机 %d moto_mod 更新为: %u", motor_index, moto_mod[motor_index]);
      }
      else if (param_type == "moto_together_flag") {
        moto_together_flag[motor_index] = static_cast<uint8_t>(param.as_int());
        RCLCPP_INFO(get_logger(), "电机 %d moto_together_flag 更新为: %u", motor_index, moto_together_flag[motor_index]);
      }
      else if (param_type == "moto_angle") {
        moto_angle[motor_index] = static_cast<double>(param.as_double());
        RCLCPP_INFO(get_logger(), "电机 %d moto_angle 更新为: %f", motor_index, moto_angle[motor_index]);
      }
    }
    return result;
  }

  // 辅助函数：从参数服务器获取所有参数值来初始化全局数组
  void update_parameters_from_server()
  {
    for (int i = 0; i < 5; i++) {
      std::string motor_prefix = "motor" + std::to_string(i) + ".";

      task_flag[i] = this->get_parameter(motor_prefix + "task_flag").as_int();
      moto_speed[i] = static_cast<uint16_t>(this->get_parameter(motor_prefix + "moto_speed").as_int());
      moto_a_speed[i] = static_cast<uint8_t>(this->get_parameter(motor_prefix + "moto_a_speed").as_int());
      moto_pose[i] = this->get_parameter(motor_prefix + "moto_pose").as_int();
      moto_mod[i] = static_cast<uint8_t>(this->get_parameter(motor_prefix + "moto_mod").as_int());
      moto_together_flag[i] = static_cast<uint8_t>(this->get_parameter(motor_prefix + "moto_together_flag").as_int());
      moto_angle[i] = static_cast<double>(this->get_parameter(motor_prefix + "moto_angle").as_double());
    }
  }
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_; // 参数回调句柄
  // 电机控制函数
  void moto_contral_location_mode(uint8_t motor_id, uint8_t direction,
    uint16_t speed, uint8_t a_speed, uint32_t pose, uint8_t mode, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(13); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0xFD;
    message.data[2] = direction;
    message.data[3] = (speed >> 8) & 0xFF; // 高字节
    message.data[4] = speed & 0xFF;        // 低字节
    message.data[5] = a_speed;
    message.data[6] = (pose >> 24) & 0xFF; // 第4字节
    message.data[7] = (pose >> 16) & 0xFF; // 第3字节
    message.data[8] = (pose >> 8) & 0xFF;  // 第2字节
    message.data[9] = pose & 0xFF;         // 第1字节
    message.data[10] = mode;
    message.data[11] = together_flag;
    message.data[12] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送位置模式指令到电机 %d", motor_id);
  }

  void moto_contral_speed_mode(uint8_t motor_id, uint8_t direction, uint16_t speed, 
                              uint8_t a_speed, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(8); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0xF6;
    message.data[2] = direction;
    message.data[3] = (speed >> 8) & 0xFF; // 高字节
    message.data[4] = speed & 0xFF;        // 低字节
    message.data[5] = a_speed;
    message.data[6] = together_flag;
    message.data[7] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送速度模式指令到电机 %d", motor_id);
  }

  void moto_contral_enable(uint8_t motor_id, uint8_t enable_flag, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(6); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0xF3;
    message.data[2] = 0xAB; // 辅助码
    message.data[3] = enable_flag;
    message.data[4] = together_flag;
    message.data[5] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送使能指令到电机 %d", motor_id);
  }

  void moto_stop_now(uint8_t motor_id, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(5); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0xFE;
    message.data[2] = 0xAB; // 辅助码
    message.data[3] = together_flag;
    message.data[4] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送急停指令到电机 %d", motor_id);
  }

  void moto_recover_zero(uint8_t motor_id, uint8_t recover_mod, uint8_t together_flag)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(5); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0x9A;
    message.data[2] = recover_mod; // 回零模式
    message.data[3] = together_flag;
    message.data[4] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送回零指令到电机 %d", motor_id);
  }

  void moto_move_together(void)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(4); // 调整数组大小
    
    message.data[0] = 0x00;
    message.data[1] = 0xFD;
    message.data[2] = 0x02;
    message.data[3] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "同步运动开始");
  }

  void moto_angle_read(uint8_t motor_id)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(3); // 调整数组大小
    
    message.data[0] = motor_id;
    message.data[1] = 0x36;
    message.data[2] = 0x6B; // 校验位
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "发送角度读取指令到电机 %d", motor_id);
  }

  void DuoJi_contral(int angle, int time)
  {
    int pwm = angle/270*2000+500;
    std::stringstream ss;
    ss << "#000P" << pwm << 'T' << time << '!';
    std::string str = ss.str();
    
    // 创建消息对象
    auto message = std_msgs::msg::String();
    message.data = str; // 设置消息内容
    
    std::cout << "字符串: " << str << std::endl;
    
    // 发布消息对象
    DuoJi_publisher_->publish(message);
  }

  void motor_init()
  {
    // 正确初始化数组
    for(int i = 0; i < 5; i++) {
      task_flag[i] = 3; // 位置模式
      moto_a_speed[i] = 100;
      moto_speed[i] = 100;
      moto_pose[i] = 0;
      last_moto_pose[i] = 0;
      moto_mod[i] = 1;
      moto_together_flag[i] = 0;
      moto_angle[i] = 0.0;
      moto_angle_now[i] = 0.0;
      last_moto_angle[i] = 0.0;
      moto_angle_sport_command_flag[i] = 1;
      moto_angle_read_command_flag[i] = 1;
      channel_idle_flag = true;
      moto_i = 0;
    }
    
    // //使能所有电机
    // for(uint8_t i = 0; i < 5; i++){

    //   moto_contral_enable(i+1, 0x01, 0x00);
    // }
  }
  bool chake_moto_angle_sport_command_flag()
  {
    for(int i = 0; i < 5; i++){
      if(moto_angle_sport_command_flag[i] != 1){
        return false;
      }
    }
    return true;
  }
  bool chake_moto_angle_read_command_flag()
  {
    for(int i = 0; i < 5; i++){
      if(moto_angle_read_command_flag[i] != 1){
        return false;
      }
    }
    return true;
  }
  void timer_callback()
  {
    if(channel_idle_flag == false)
    {
      // return; // 通道忙，跳过此次定时器回调
      waiting_count++;
      if(waiting_count<20) return; // 通道忙，跳过此次定时器回调
      else channel_idle_flag = true; // 等待超时，强制通道空闲
    }
    waiting_count=0;
    // for(uint8_t i = 0; i < 6; i++){
      switch (task_flag[moto_i])
      {
        case 0: // 急停
          moto_stop_now(moto_i+1, moto_together_flag[moto_i]);
          break;
        case 1: // 监听模式
          // 不发送控制指令，只监听
          moto_angle_read(moto_i+1);
          break;
        case 2: // 使能模式
          moto_contral_enable(moto_i+1, 0x01, moto_together_flag[moto_i]);
          break;
        case 3: // 位置模式
          if(channel_idle_flag)
          {
            // RCLCPP_WARN(this->get_logger(), "未知任务标志                                           : %d", moto_i+1);
            if(std::fabs(last_moto_pose[moto_i]-moto_pose[moto_i])>0.001){
              moto_angle[moto_i] = static_cast<uint32_t>(moto_pose[moto_i]/3200/50*360); // 计算角度
              last_moto_pose[moto_i] = moto_pose[moto_i]; // 更新上一个位置
              last_moto_angle[moto_i] = moto_angle[moto_i]; // 更新上一个角度
              moto_contral_location_mode(moto_i+1, moto_derection[moto_i], moto_speed[moto_i], moto_a_speed[moto_i], moto_pose[moto_i], moto_mod[moto_i], moto_together_flag[moto_i]);
              moto_angle_sport_command_flag[moto_i] = 0; // 运动命令已改变，等待发送结果
              channel_idle_flag = false; // 通道忙
            }
            else if(std::fabs(last_moto_angle[moto_i]-moto_angle[moto_i])>0.001){
              // RCLCPP_WARN(this->get_logger(), "aaaa");
              moto_pose[moto_i] = static_cast<uint32_t>(moto_angle[moto_i]/360*50*3200); // 计算位置
              last_moto_angle[moto_i] = moto_angle[moto_i]; // 更新上一个角度
              last_moto_pose[moto_i] = moto_pose[moto_i]; // 更新上一个位置
              moto_contral_location_mode(moto_i+1, moto_derection[moto_i], moto_speed[moto_i], moto_a_speed[moto_i], moto_pose[moto_i], moto_mod[moto_i], moto_together_flag[moto_i]);
              moto_angle_sport_command_flag[moto_i] = 0; // 运动命令已改变，等待发送结果
              channel_idle_flag = false; // 通道忙
            }
            else if(std::fabs(moto_angle_now[moto_i] - moto_angle[moto_i]) > 0.001){
              // RCLCPP_WARN(this->get_logger(), "未知任务标志: %d", moto_i+1);
              // RCLCPP_WARN(this->get_logger(), "未知任务标志: %.2f   %.2f", moto_angle_now[moto_i],moto_angle[moto_i]);
              moto_angle_read(moto_i+1);
              moto_angle_read_command_flag[moto_i] = 0;
              channel_idle_flag = false;
            }
          }
          break;
        case 4: // 速度模式
          moto_contral_speed_mode(moto_i+1, moto_derection[moto_i], moto_speed[moto_i], moto_a_speed[moto_i], moto_together_flag[moto_i]);
          break;
        case 5: // 回零模式
          moto_recover_zero(moto_i+1, moto_derection[moto_i], moto_together_flag[moto_i]);
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "未知任务标志: %d for motor %d", task_flag[moto_i], moto_i);
          break;
      }
      if(moto_i<4){
        moto_i++;
      }else{
        moto_i=0;
      }
      // for(int j=0;j<5;j++){
      //   RCLCPP_INFO(this->get_logger(), "电机: %d sport_flag: %d read_flag: %d", j,moto_angle_sport_command_flag[j], moto_angle_sport_command_flag[j]);
      // }

      std_msgs::msg::Float64MultiArray moto_angle_message;
      moto_angle_message.data = std::vector<double>(moto_angle_now, moto_angle_now + 5);
      moto_angle_publisher_->publish(moto_angle_message);

      std::stringstream ss;
      // if(chake_moto_angle_read_command_flag())
      // {
      ss << "电机角度: ";
      for(int i = 0; i < 5; i++) {
        ss << i << ":" << std::fixed << std::setprecision(2) << moto_angle_now[i] << "度 ";
      }
      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
  //    }
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