import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

namespace_ = 'controller'

def generate_launch_description():
    
    kinematics = Node(
        package='controller',
        executable='Kinematics_node',
        name='Kinematics_node',
        namespace=namespace_,
        output='screen'
    )
    motors = Node(
        package='controller',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        output='screen'
    )
    
    
    return LaunchDescription([
        kinematics,
        motors
    ])
