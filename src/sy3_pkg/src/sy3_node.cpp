#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <iostream>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("sy3_node"), serial_fd_(-1)
  {
    // 声明参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    
    // 获取参数值
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    
    // 初始化串口
    if (!init_serial(port, baud)) {
      RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
      throw std::runtime_error("串口初始化失败");
    }
    
    // 创建订阅者，接收要发送的数据
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "serial_tx", 10, std::bind(&SerialNode::serial_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "串口节点已启动，监听话题: /serial_tx");
  }
  
  ~SerialNode()
  {
    // 关闭串口
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
  }

private:
  bool init_serial(const std::string& port, int baud_rate)
  {
    // 打开串口设备
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", port.c_str());
      return false;
    }
    
    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "无法获取串口属性");
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    
    // 设置波特率
    speed_t speed;
    switch (baud_rate) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: 
        RCLCPP_WARN(this->get_logger(), "不支持的波特率: %d, 使用默认值 115200", baud_rate);
        speed = B115200;
        break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8位数据位，无奇偶校验，1位停止位
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_cflag |= (CLOCAL | CREAD);
    
    // 禁用软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    
    // 设置超时
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5; // 0.5秒超时
    
    // 应用设置
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "无法设置串口属性");
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    
    // 清空输入输出缓冲区
    tcflush(serial_fd_, TCIOFLUSH);
    
    RCLCPP_INFO(this->get_logger(), "成功打开串口: %s, 波特率: %d", port.c_str(), baud_rate);
    return true;
  }
  
  void serial_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "串口未打开，无法发送数据");
      return;
    }
    
    // 发送数据到串口
    ssize_t bytes_written = write(serial_fd_, msg->data.c_str(), msg->data.length());
    
    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "串口写入错误");
    } else {
      RCLCPP_INFO(this->get_logger(), "已发送 %zd 字节: %s", bytes_written, msg->data.c_str());
    }
  }
  
  int serial_fd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    rclcpp::spin(std::make_shared<SerialNode>());
  } catch (const std::exception& e) {
    std::cerr << "节点运行异常: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}