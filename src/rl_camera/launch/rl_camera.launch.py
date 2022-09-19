import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

namespace_ = 'rl_camera'

def generate_launch_description():
    
    camera = Node(
        package='rl_camera',
        executable='Camera_node',
        name='Camera_node',
        namespace=namespace_,
        output='screen'
    )
    
    model = Node(
        package='rl_camera',
        executable='RLModel_node',
        name='RLModel_node',
        namespace=namespace_,
        output='screen'
    )
    
    return LaunchDescription([
        camera,
        model
    ])
