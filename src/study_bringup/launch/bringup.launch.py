"""一条命令拉起整个 5 轴机械臂栈。

用法:
    ros2 launch study_bringup bringup.launch.py
    ros2 launch study_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
    ros2 launch study_bringup bringup.launch.py enable_websocket:=false

启动的节点:
    - sy4_pkg/sy4_node           串口驱动 (与 STM32 通信)
    - sy5_pkg/sy5_node           5 轴电机 + 舵机控制
    - sy6_pkg/sy6_node           正运动学解算
    - sy7_pkg/sy7_node           逆运动学解算
    - tip_position_bridge/tip_position_client  WebSocket 桥 (可关)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    enable_checksum = LaunchConfiguration('enable_checksum')
    axle3_target_angle = LaunchConfiguration('axle3_target_angle')
    pc_ip = LaunchConfiguration('pc_ip')
    websocket_port = LaunchConfiguration('websocket_port')
    heartbeat_interval = LaunchConfiguration('heartbeat_interval')
    enable_websocket = LaunchConfiguration('enable_websocket')
    log_level = LaunchConfiguration('log_level')

    declared_args = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial device path used by sy4_node.'),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate used by sy4_node.'),
        DeclareLaunchArgument(
            'enable_checksum',
            default_value='false',
            description='Whether sy4_node should validate the trailing checksum.'),
        DeclareLaunchArgument(
            'axle3_target_angle',
            default_value='0.0',
            description='Target angle (deg) for joint 3 in sy7_node IK solver.'),
        DeclareLaunchArgument(
            'pc_ip',
            default_value='172.20.10.3',
            description='Host running rosbridge_websocket the bridge connects to.'),
        DeclareLaunchArgument(
            'websocket_port',
            default_value='9090',
            description='rosbridge_websocket port.'),
        DeclareLaunchArgument(
            'heartbeat_interval',
            default_value='5.0',
            description='Heartbeat interval (s) for the WebSocket bridge.'),
        DeclareLaunchArgument(
            'enable_websocket',
            default_value='true',
            description='Set to false to skip the tip_position_bridge node.'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level applied to all nodes (debug/info/warn/error).'),
    ]

    log_arguments = ['--ros-args', '--log-level', log_level]

    sy4_node = Node(
        package='sy4_pkg',
        executable='sy4_node',
        name='sy4_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'enable_checksum': enable_checksum,
        }],
        arguments=log_arguments,
    )

    sy5_node = Node(
        package='sy5_pkg',
        executable='sy5_node',
        name='sy5_node',
        output='screen',
        arguments=log_arguments,
    )

    sy6_node = Node(
        package='sy6_pkg',
        executable='sy6_node',
        name='sy6_node',
        output='screen',
        arguments=log_arguments,
    )

    sy7_node = Node(
        package='sy7_pkg',
        executable='sy7_node',
        name='sy7_node',
        output='screen',
        parameters=[{
            'axle3_target_angle': axle3_target_angle,
        }],
        arguments=log_arguments,
    )

    websocket_bridge = GroupAction(
        condition=IfCondition(enable_websocket),
        actions=[
            Node(
                package='tip_position_bridge',
                executable='tip_position_client',
                name='tip_position_client',
                output='screen',
                parameters=[{
                    'pc_ip': pc_ip,
                    'websocket_port': websocket_port,
                    'heartbeat_interval': heartbeat_interval,
                }],
                arguments=log_arguments,
            ),
        ],
    )

    return LaunchDescription([
        *declared_args,
        sy4_node,
        sy5_node,
        sy6_node,
        sy7_node,
        websocket_bridge,
    ])
