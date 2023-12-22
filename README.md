# scout_nav2

AgileX Scout robot for navigation with Nav2, in the context of mobile manipulation - ROS2

Author: Simone Giampà

Project realized at Politecnico di Milano as Master's Thesis in Computer Science and Engineering

Academic Year: 2023/2024

## Description

This repository contains the configuration files and launch files for the navigation of the AgileX Scout robot with Nav2. It also contains the configuration files for the simulation of the robot in Gazebo.

## Installation

### Dependencies

- navigation2
- nav2-bringup
- pointcloud-to-laserscan
- slam-toolbox
- ros-gzfortress
- ignition-gazebo6

### Building

Build the package with `colcon build`.

### Usage

1. First launch the robot with the lidar sensor, either in simulation or the real robot:

   * To launch the simulation of the robot in Gazebo, run the following command:

`ros2 launch agilex_scout simulate_control_gazebo.launch.py lidar_type:=<3d|2d> rviz:=<true|false>`

> - lidar_type: 3d for a 3D lidar (pointcloud2), 2d for a 2D lidar (laserscan)
> - rviz: true to launch rviz, false to not launch rviz gui

  * To launch the real scout robot with the lidar sensor and the laserscan conversion node, run the following command:

`ros2 launch agilex_scout scout_robot_lidar.launch.py load_gazebo:=<true|false>`

> - load_gazebo: true to load also the gazebo simulation environment, false to load only the RViz display gui

2. Second step: launch the entire navigation2 stack, run the following command:

`ros2 launch scout_nav2 nav2.launch.py simulation:=<true|false> slam:=<True|False>`

> - slam: True if you want to use SLAM toolbox for map creation, False if you want to use AMCL for localizaion + nav2 for mapping
> - simulation: true if running in simulation with gazebo, false if launching the real AgileX Scout robot with real sensors
