import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'pc_ip',
            default_value='172.20.10.3',
            description='IP address of the PC running Rosbridge'
        ),
        DeclareLaunchArgument(
            'websocket_port',
            default_value='9090',
            description='WebSocket port of Rosbridge server'
        ),
        DeclareLaunchArgument(
            'heartbeat_interval',
            default_value='5.0',
            description='Heartbeat interval in seconds'
        ),
        
        Node(
            package='tip_position_bridge',
            executable='tip_position_client',
            name='tip_position_client',
            output='screen',
            parameters=[{
                'pc_ip': LaunchConfiguration('pc_ip'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'heartbeat_interval': LaunchConfiguration('heartbeat_interval'),
            }]
        ),
    ])