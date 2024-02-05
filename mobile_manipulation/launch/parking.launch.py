import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # start the demo autonomy task
    navigate_demo_node = Node(
        name="parking_demo",
        package="scout_nav2",
        executable="park_robot",
        emulate_tty=True,
        output="screen",
    )

    return LaunchDescription([navigate_demo_node])
