import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

namespace_ = 'controller'

def generate_launch_description():
    return LaunchDescription([
        
        Node(
           package='controller',
           namespace='motor_subscriber',
           executable='motor_subscriber',
           name='motor_subscriber'
        )
        
    ])