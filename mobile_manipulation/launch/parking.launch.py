
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # start the demo autonomy task
    navigate_demo_node = Node(
        name="parking_demo",
        package="mobile_manipulation",
        executable="parking_demo.py",
        emulate_tty=True,
        output="screen",
    )

    return LaunchDescription([navigate_demo_node])
