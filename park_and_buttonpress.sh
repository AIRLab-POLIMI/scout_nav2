#!/bin/bash

# Function to execute a ROS 2 launch file in a new Konsole terminal
run_ros2_launch_in_konsole() {
    konsole --hold -e bash -c "ros2 launch $1"
}

run_ros2_launch_in_konsole "mobile_manipulation multiple_aruco_nodes.launch.py" & sleep 3

# Spawn Konsole terminals for each ROS 2 launch file
run_ros2_launch_in_konsole "igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=simulation load_base:=true" & sleep 3

run_ros2_launch_in_konsole "agilex_scout scout_robot_lidar.launch.py"  & sleep 4

run_ros2_launch_in_konsole "scout_nav2 nav2.launch.py simulation:=false slam:=False localization:=slam_toolbox"  & sleep 6

run_ros2_launch_in_konsole "mobile_manipulation navigate_and_button_press_demo.launch.py"  & sleep 5







# You can add more launch files as needed
exit 0
