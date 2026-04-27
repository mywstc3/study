#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include "std_msgs/msg/float64_multi_array.hpp"
const double PI = std::acos(-1.0);
struct Axle
{
  double axle_long;
  double progection_x;
  double progection_y;
  double progection_z;
  double angle_a;
  double angle_last;
};

Axle axle[6];
double last_target_x,last_target_y;
class Location_Resolve_Node : public rclcpp::Node
{
public:
  Location_Resolve_Node() : Node("sy7_node")
  {
    
    location_target_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("location_target", 10, std::bind(&Location_Resolve_Node::location_target_callback, this, std::placeholders::_1));
    
    moto_angle_target_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("moto_target_angle", 10);
    
    RCLCPP_INFO(this->get_logger(), "逆解算节点已启动");
    // location_target_angle_solve(300.0,0.0,100);
  }
  
  ~Location_Resolve_Node()
  {
    RCLCPP_INFO(this->get_logger(), "逆解算节点已关闭");
  }

private:
  
  
  void location_target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    double target_x = msg->data[0];
    double target_y = msg->data[1];
    double target_z = msg->data[2];
    location_target_angle_solve(target_x,target_y,target_z);
    std_msgs::msg::Float64MultiArray message;
    message.data.resize(6); // 调整数组大小
    for(int i = 0; i < 6; i++) {
      message.data[i] = axle[i].angle_a;
    }
    moto_angle_target_publisher->publish(message);
  }
  bool judge_angle()
  {
    for(int i=0;i<5;i++){
      if(axle[i].angle_a>180 || axle[i].angle_a<0 || std::isnan(axle[i].angle_a)){
        RCLCPP_ERROR(this->get_logger(), "电机 %d 角度超出范围: %2f", i,axle[i].angle_a);
        return false;
      }

    }
    return true;
  }
  void location_target_angle_solve(double target_x,double target_y, double target_z)
  {
    axle[0].axle_long=130.5, axle[1].axle_long=40.0, axle[2].axle_long=180.0, axle[3].axle_long=168.0, axle[4].axle_long=127.5, axle[5].axle_long=143.0;
    double target_progection_x = sqrt(target_x*target_x + target_y*target_y);

    // target_z = target_z+10;
    // double target_progection_x_p = sqrt(target_x*target_x + target_y*target_y);
    // double target_progection_x = target_progection_x_p+10;
    // target_x = target_progection_x/target_progection_x_p*target_x;
    // target_y = target_progection_x/target_progection_x_p*target_y;
    axle[0].progection_x = 0;
    axle[0].progection_y = 0;
    axle[0].progection_z = axle[0].axle_long;
    // RCLCPP_INFO(get_logger(), "电机 %2f ,%2f,%2f", target_progection_x,target_x,target_y);
    axle[1].progection_x = 0;
    axle[1].progection_y = 0;
    axle[1].progection_z = axle[0].axle_long+axle[1].axle_long;

    axle[5].progection_x = target_progection_x;
    axle[5].progection_y = 0;
    axle[5].progection_z = target_z;

    axle[4].progection_x = axle[5].progection_x;
    axle[4].progection_y = 0;
    axle[4].progection_z = axle[5].progection_z+axle[5].axle_long;

    axle[0].angle_a = (PI-atan2(target_y,target_x))/PI*180;
    // RCLCPP_INFO(get_logger(), " %2f ,%2f,%2f", atan2(target_y,target_x),target_x,target_y);
    double axle_24_long = sqrt(axle[3].axle_long*axle[3].axle_long + axle[4].axle_long*axle[4].axle_long);
    double axle_14_long = sqrt(std::abs(axle[1].progection_z-axle[4].progection_z)*std::abs(axle[1].progection_z-axle[4].progection_z)+axle[4].progection_x*axle[4].progection_x);
    double axle_15_long = sqrt(std::abs(axle[1].progection_z-axle[5].progection_z)*std::abs(axle[1].progection_z-axle[5].progection_z)+axle[5].progection_x*axle[5].progection_x);
    double angle_41p = atan2(axle[4].progection_z-axle[1].progection_z,axle[4].progection_x);
    double angle_214 = acos((axle_14_long*axle_14_long+axle[2].axle_long*axle[2].axle_long-axle_24_long*axle_24_long)/(2*axle_14_long*axle[2].axle_long));
    axle[3].angle_a = 0;
    axle[1].angle_a = 180-(angle_41p+angle_214)/PI*180;

    double angle_124 = acos((axle[2].axle_long*axle[2].axle_long+axle_24_long*axle_24_long-axle_14_long*axle_14_long)/(2*axle[2].axle_long*axle_24_long));
    double angle_423 = atan2(axle[4].axle_long,axle[3].axle_long);
    axle[2].angle_a = (angle_124+angle_423)/PI*180 - 90;

    double angle_243 = atan2(axle[3].axle_long,axle[4].axle_long);
    double angle_142 = acos((axle_14_long*axle_14_long+axle_24_long*axle_24_long-axle[2].axle_long*axle[2].axle_long)/(2*axle_14_long*axle_24_long));
    double angle_145 = acos((axle_14_long*axle_14_long+axle[5].axle_long*axle[5].axle_long-axle_15_long*axle_15_long)/(2*axle_14_long*axle[5].axle_long));
    axle[4].angle_a = 270 - (angle_243+angle_142+angle_145)/PI*180; 

    if(!judge_angle()){
      RCLCPP_ERROR(this->get_logger(), "位置不可达,使用上一次位置");
      for(int j=0;j<5;j++){
        axle[j].angle_a=axle[j].angle_last;
      }
    }
    else{
      for(int j=0;j<5;j++){
        axle[j].angle_last=axle[j].angle_a;
      }
    }

    std::stringstream ss;
    ss << "电机角度: ";
    for(int i = 0; i < 6; i++) {
      ss << i << ":" << std::fixed << std::setprecision(2) << axle[i].angle_a << "度 ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    // RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]]", axle_14_long, angle_142/PI*180, axle_24_long);
  }
  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr location_target_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr moto_angle_target_publisher;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Location_Resolve_Node>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}