#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <vector>
#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("sy4_node"), serial_fd_(-1), running_(false), 
                 parser_state_(STATE_WAIT_HEADER), last_byte_time_(std::chrono::steady_clock::now())
  {
    // 声明参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    // this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<bool>("enable_checksum", false); // 是否启用校验和
    
    // 获取参数值
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    enable_checksum_ = this->get_parameter("enable_checksum").as_bool();
    
    // 初始化串口
    if (!init_serial(port, baud)) {
      RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
      throw std::runtime_error("串口初始化失败");
    }
    
    // 创建订阅者，接收UInt8MultiArray数据并发送到串口
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_tx", 20, std::bind(&SerialNode::serial_tx_callback, this, std::placeholders::_1));
    
    DuoJi_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "DuoJi_serial_tx", 20, std::bind(&SerialNode::DuoJi_serial_tx_callback, this, std::placeholders::_1));
    
    // 创建发布者，用于发布从串口接收到的数据
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_rx", 10);
    
    // 启动串口接收线程
    running_.store(true);
    receive_thread_ = std::thread(&SerialNode::serial_receive_loop, this);
    
    RCLCPP_INFO(this->get_logger(), "串口节点已启动");
    RCLCPP_INFO(this->get_logger(), "监听话题: /serial_tx (发送数据到串口)");
    RCLCPP_INFO(this->get_logger(), "发布话题: /serial_rx (从串口接收数据)");
    RCLCPP_INFO(this->get_logger(), "数据包格式: 包头(0x01/0x02) + 数据 + 包尾(0x6B)");
    if (enable_checksum_) {
      RCLCPP_INFO(this->get_logger(), "校验和验证: 已启用");
    }
  }
  
  ~SerialNode()
  {
    // 停止接收线程
    running_.store(false);
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    
    // 关闭串口
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
  }

private:
  // 解析器状态
  enum ParserState {
    STATE_WAIT_HEADER,    // 等待包头
    STATE_IN_PACKET,      // 在数据包中
    STATE_CHECK_TAIL      // 检查包尾
  };
  
  bool init_serial(const std::string& port, int baud_rate)
  {
    // 打开串口设备
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开: %s", port.c_str());
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
    tcflush(serial_fd_, TCIFLUSH);
    
    RCLCPP_INFO(this->get_logger(), "成功打开串口: %s, 波特率: %d", port.c_str(), baud_rate);
    return true;
  }
  void DuoJi_serial_tx_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "串口未打开，无法发送数据");
      return;
    }
    
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "接收到空数据数组");
      return;
    }
    
    ssize_t bytes_written = write(serial_fd_, msg->data.data(), msg->data.size());
    
    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "串口写入错误");
    } else {
      RCLCPP_INFO(this->get_logger(), "已发送 %zd 字节", bytes_written);
    }                                                    
  }
  void serial_tx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "串口未打开，无法发送数据");
      return;
    }
    
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "接收到空数据数组");
      return;
    }
    
    ssize_t bytes_written = write(serial_fd_, msg->data.data(), msg->data.size());
    
    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "串口写入错误");
    } else {
      RCLCPP_INFO(this->get_logger(), "已发送 %zd 字节", bytes_written);
    }
  }
  
  // 计算校验和（简单的异或校验）
  uint8_t calculate_checksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
      checksum ^= byte;
    }
    return checksum;
  }
  
  // 处理接收到的数据字节
  void process_received_byte(uint8_t byte) {
    auto now = std::chrono::steady_clock::now();

    // 检查超时（100ms无新数据则重置解析器）
    // 注意：必须先用旧的 last_byte_time_ 比较，再更新它，否则差值永远是 0
    if (parser_state_ != STATE_WAIT_HEADER &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_byte_time_).count() > 100) {
      RCLCPP_WARN(this->get_logger(), "数据包接收超时，已丢弃 %zu 字节", packet_buffer_.size());
      reset_parser();
      last_byte_time_ = now;
      return;
    }

    last_byte_time_ = now;
    
    switch (parser_state_) {
      case STATE_WAIT_HEADER:
        // 检查是否是有效的包头（0x01或0x02）
        if (byte == 0x01 || byte == 0x02 || byte == 0x03 || byte == 0x04 || byte == 0x05) {
          packet_buffer_.clear();
          packet_buffer_.push_back(byte);
          parser_state_ = STATE_IN_PACKET;
          RCLCPP_DEBUG(this->get_logger(), "检测到包头: 0x%02X", byte);
        }
        break;
        
      case STATE_IN_PACKET:
        // 检查是否是包尾（0x6B）
        if (byte == 0x6B) {
          packet_buffer_.push_back(byte);
          parser_state_ = STATE_CHECK_TAIL;
          RCLCPP_DEBUG(this->get_logger(), "检测到包尾: 0x%02X", byte);
        } else {
          // 普通数据字节
          packet_buffer_.push_back(byte);
        }
        break;
        
      case STATE_CHECK_TAIL:
        // 完整数据包接收完成
        if (packet_buffer_.size() >= 2) { // 至少包含包头和包尾
          // 验证校验和（如果启用）
          bool valid_packet = true;
          if (enable_checksum_ && packet_buffer_.size() > 3) {
            // 假设校验和是倒数第二个字节
            uint8_t received_checksum = packet_buffer_[packet_buffer_.size() - 2];
            std::vector<uint8_t> data_to_check(packet_buffer_.begin(), packet_buffer_.end() - 2);
            uint8_t calculated_checksum = calculate_checksum(data_to_check);
            
            if (received_checksum != calculated_checksum) {
              RCLCPP_WARN(this->get_logger(), "校验和错误: 接收=0x%02X, 计算=0x%02X", 
                         received_checksum, calculated_checksum);
              valid_packet = false;
            }
          }
          
          if (valid_packet) {
            // 发布完整数据包
            auto message = std_msgs::msg::UInt8MultiArray();
            message.data = packet_buffer_;
            publisher_->publish(message);
            
            RCLCPP_INFO(this->get_logger(), "接收到完整数据包，长度: %zu", packet_buffer_.size());
            
            // 打印数据包内容（调试用）
            std::string hex_data;
            for (size_t i = 0; i < packet_buffer_.size(); ++i) {
              char hex[8];
              snprintf(hex, sizeof(hex), "0x%02X ", packet_buffer_[i]);
              hex_data += hex;
            }
            RCLCPP_DEBUG(this->get_logger(), "数据包: %s", hex_data.c_str());
          }
        }
        
        // 重置解析器，等待下一个数据包
        reset_parser();
        
        // 检查当前字节是否是新的包头
        if (byte == 0x01 || byte == 0x02 || byte == 0x03 || byte == 0x04 || byte == 0x05) {
          packet_buffer_.push_back(byte);
          parser_state_ = STATE_IN_PACKET;
          RCLCPP_DEBUG(this->get_logger(), "检测到新包头: 0x%02X", byte);
        }
        break;
    }
  }
  
  // 重置解析器状态
  void reset_parser() {
    parser_state_ = STATE_WAIT_HEADER;
    packet_buffer_.clear();
  }
  
  void serial_receive_loop() {
    std::vector<uint8_t> buffer(1024);
    
    while (running_.load()) {
      if (serial_fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      
      ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
      
      if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", strerror(errno));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      
      if (bytes_read > 0) {
        RCLCPP_DEBUG(this->get_logger(), "原始接收 %zd 字节", bytes_read);
        
        // 处理每个接收到的字节
        for (ssize_t i = 0; i < bytes_read; ++i) {
          process_received_byte(buffer[i]);
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
  
  int serial_fd_;
  std::atomic<bool> running_;
  std::thread receive_thread_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr DuoJi_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  
  // 数据包解析相关变量
  ParserState parser_state_;
  std::vector<uint8_t> packet_buffer_;
  std::chrono::steady_clock::time_point last_byte_time_;
  bool enable_checksum_;
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