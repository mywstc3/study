#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class LocationTargetPublisher(Node):
    def __init__(self):
        super().__init__('location_target_publisher')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/location_target', 
            10
        )
        
        self.get_logger().info("位置目标发布器已启动")
        self.get_logger().info("请输入 x y z 坐标（用空格分隔，例如：'1.0 2.0 0.5'）")
        
        # 启动输入循环
        self.timer = self.create_timer(0.1, self.read_input)

    def read_input(self):
        """读取用户输入并发布坐标"""
        try:
            # 获取用户输入（非阻塞方式）
            input_str = input("> ")
            
            # 尝试解析输入
            try:
                coords = list(map(float, input_str.split()))
                if len(coords) != 3:
                    raise ValueError("需要3个坐标值")
                
                # 创建并发布消息
                msg = Float64MultiArray()
                msg.data = coords
                self.publisher.publish(msg)
                self.get_logger().info(f"已发布: {coords}")
                
            except ValueError as e:
                self.get_logger().error(f"输入错误: {e}")
                self.get_logger().info("请重新输入 x y z 坐标（用空格分隔）")
                
        except EOFError:
            # 处理 Ctrl+D 退出
            self.get_logger().info("检测到输入结束，退出程序...")
            rclpy.shutdown()
        except KeyboardInterrupt:
            # 处理 Ctrl+C 退出
            self.get_logger().info("用户中断，退出程序...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LocationTargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()