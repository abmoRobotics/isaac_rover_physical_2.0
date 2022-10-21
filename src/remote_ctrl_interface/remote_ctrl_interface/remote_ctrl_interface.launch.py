import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

namespace_ = 'remote_ctrl_interface'


def generate_launch_description():
    
    live_feed = Node(
        package='remote_ctrl_interface',
        executable='live_feed',
        name='live_feed',
        namespace=namespace_,
        output='screen'
    )

    return LaunchDescription([
        live_feed,
    ])