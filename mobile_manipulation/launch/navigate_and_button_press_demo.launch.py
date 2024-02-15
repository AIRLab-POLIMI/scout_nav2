""" Navigate and button press demo

Version 1: assuming target aruco spot already located, received as /target_pose pose topic, sent with rviz2 button in gui
Version 2: rotate the arm with the camera on top, and spin around itself, until the target aruco spot is located.
            Once the aruco is located, it will publish the estimated location on /target_pose pose topic, starting the demo

0. given target_pose received with pose topic
1. start park_robot.py --> parking_algorithm_then_navigate() with subscription callback to /target_pose
2. computes parking pose, then navigate to /target_goal. Assuming target is reached, the demo continues, otherwise the demo fails
3. starts button_press_demo code --> lookAroundForArucoMarkers searching motion --> buttonPressingSequence demo

Architecture

demo C++ client:
- client sends request of high-level motion commands to the C++ button presser server
- client sends request of high-level parking and navigation commands to the python parking server

button presser C++ server:
- handle request for looking around for aruco markers and returns once they are found
- handle request for pressing detected aruco markers and returns once the button pressing demo is finished

parking python server:
- handle request for computing parking position and navigation to the target goal using NAV2, returns once the target goal is reached or the navigation aborts

"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_parking_action_server = Node(
        package='mobile_manipulation',
        executable='robot_parking_action_server.py',
        name='robot_parking_action_server',
        output='screen',
        emulate_tty=True,
        #parameters=[{'use_sim_time': use_sim_time}]
    )

    parking_and_interact_client = Node(
        package='mobile_manipulation',
        executable='park_and_interact',
        name='park_and_interact',
        output='screen',
        emulate_tty=True,
        #parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        robot_parking_action_server,
        parking_and_interact_client
    ])