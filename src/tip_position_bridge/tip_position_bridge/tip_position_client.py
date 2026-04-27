#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 添加类型提示忽略
# type: ignore

import rclpy
from rclpy.node import Node
import websocket  # type: ignore
import json
import time
import logging
import threading
from std_msgs.msg import Float64MultiArray, String

# 配置日志
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)
logger = logging.getLogger("TipPositionClient")

class TipPositionClient(Node):
    def __init__(self):
        super().__init__('tip_position_client_node')
        
        # 声明参数
        self.declare_parameter('pc_ip', '172.20.10.3')
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('heartbeat_interval', 5.0)
        
        # 获取参数值
        self.pc_ip = self.get_parameter('pc_ip').value
        self.websocket_port = self.get_parameter('websocket_port').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        
        self.ws_url = f"ws://{self.pc_ip}:{self.websocket_port}"
        self.ws = None
        self.connected = False
        self.reconnect_count = 0
        self.max_reconnect = 10
        
        # 创建ROS发布者 - 用于发布从客户端接收的目标位置
        self.location_target_pub = self.create_publisher(
            Float64MultiArray, 
            '/location_target', 
            10
        )
        
        # 创建ROS订阅者 - 用于订阅当前末端位置
        self.location_now_sub = self.create_subscription(
            Float64MultiArray, 
            '/location_now', 
            self.location_now_callback, 
            10
        )
        
        # 连接WebSocket
        self.connect_websocket()
        
        # 启动心跳线程
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        logger.info("末端位置通信节点已启动")
        logger.info(f"WebSocket URL: {self.ws_url}")
    
    def connect_websocket(self):
        """连接到电脑端的Rosbridge"""
        if self.reconnect_count >= self.max_reconnect:
            logger.error("达到最大重连次数，停止重连")
            return
            
        try:
            logger.info(f"正在连接到: {self.ws_url}")
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            
            self.ws_thread = threading.Thread(target=self.ws.run_forever)
            self.ws_thread.daemon = True
            self.ws_thread.start()
            
        except Exception as e:
            logger.error(f"连接失败: {str(e)}")
            self.reconnect_count += 1
            time.sleep(2)
            self.connect_websocket()
    
    def on_open(self, ws):
        """连接成功回调"""
        logger.info("连接成功!")
        self.connected = True
        self.reconnect_count = 0  # 重置重连计数
        
        # 订阅电脑端的目标位置话题
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/location_target",
            "type": "std_msgs/Float64MultiArray"
        }
        ws.send(json.dumps(subscribe_msg))
        
        # 注册发布的话题 - 用于发送当前位置
        topics_to_advertise = [
            "/location_now",
            "/tip_status"
        ]
        
        for topic in topics_to_advertise:
            advertise_msg = {
                "op": "advertise",
                "topic": topic,
                "type": "std_msgs/Float64MultiArray"
            }
            ws.send(json.dumps(advertise_msg))
        
        # 发送连接成功消息
        self.send_status("connected")
    
    def on_message(self, ws, message):
        """处理接收到的消息"""
        try:
            data = json.loads(message)
            
            # 处理目标位置消息
            if data.get("op") == "publish" and data["topic"] == "/location_target":
                target_data = data["msg"]["data"]
                logger.info(f"收到目标位置消息: {target_data}")
                
                # 发布目标位置到ROS系统
                self.publish_target_location(target_data)
            
            # 处理其他控制消息
            elif data.get("op") == "publish" and data["topic"] == "/control_command":
                command = data["msg"]["data"]
                logger.info(f"收到控制命令: {command}")
                self.handle_control_command(command)
                
        except json.JSONDecodeError:
            logger.error(f"消息不是有效的JSON: {message}")
        except Exception as e:
            logger.error(f"处理消息出错: {str(e)}")
    
    def location_now_callback(self, msg):
        """接收来自末端的当前位置"""
        logger.info(f"收到末端当前位置: {msg.data}")
        
        # 转发当前位置到客户端
        self.send_location_now(msg.data)
    
    def publish_target_location(self, target_data):
        """发布目标位置到ROS系统"""
        try:
            ros_msg = Float64MultiArray()
            ros_msg.data = target_data
            self.location_target_pub.publish(ros_msg)
            logger.info(f"已发布目标位置到ROS: {target_data}")
        except Exception as e:
            logger.error(f"发布目标位置失败: {str(e)}")
    
    def send_location_now(self, location_data):
        """发送当前位置到客户端"""
        if self.connected and self.ws:
            try:
                publish_msg = {
                    "op": "publish",
                    "topic": "/location_now",
                    "msg": {"data": location_data}
                }
                self.ws.send(json.dumps(publish_msg))
                logger.info(f"发送当前位置到客户端: {location_data}")
            except Exception as e:
                logger.error(f"发送位置消息失败: {str(e)}")
                self.connected = False
        else:
            logger.warning("未连接，无法发送位置消息")
    
    def send_status(self, status_msg):
        """发送状态消息到客户端"""
        if self.connected and self.ws:
            try:
                # 将状态消息转换为Float64MultiArray格式
                status_data = [float(ord(c)) for c in status_msg] if status_msg else [0.0]
                
                publish_msg = {
                    "op": "publish",
                    "topic": "/tip_status",
                    "msg": {"data": status_data}
                }
                self.ws.send(json.dumps(publish_msg))
                logger.info(f"发送状态消息: {status_msg}")
            except Exception as e:
                logger.error(f"发送状态消息失败: {str(e)}")
                self.connected = False
    
    def handle_control_command(self, command):
        """处理控制命令"""
        # 这里可以根据需要添加处理控制命令的逻辑
        # 例如：启动、停止、暂停等
        logger.info(f"处理控制命令: {command}")
        
        # 示例：将控制命令发布到ROS系统
        try:
            control_pub = self.create_publisher(String, '/control_command', 10)
            ros_msg = String()
            ros_msg.data = command
            control_pub.publish(ros_msg)
        except Exception as e:
            logger.error(f"发布控制命令失败: {str(e)}")
    
    def send_heartbeat(self):
        """发送心跳包保持连接"""
        while True:
            if self.connected:
                try:
                    # 发送心跳状态
                    self.send_status("heartbeat")
                except:
                    self.connected = False
            time.sleep(self.heartbeat_interval)
    
    def on_error(self, ws, error):
        logger.error(f"WebSocket错误: {error}")
        self.connected = False
    
    def on_close(self, ws, close_status_code, close_msg):
        logger.warning(f"连接关闭 (状态码: {close_status_code}, 信息: {close_msg})")
        self.connected = False
        time.sleep(2)
        self.connect_websocket()  # 自动重连

def main(args=None):
    rclpy.init(args=args)
    
    client = TipPositionClient()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        logger.info("用户中断，退出程序")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()