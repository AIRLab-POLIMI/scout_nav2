# AgileX Scout Navigation with Nav2 and Simulation with Ignition Gazebo v6 (Fortress)

Autonomous navigation using the AgileX Scout mobile wheeled robot using NAV2 & ROS2

Author: __Simone Giampà__

Project realized at __Politecnico di Milano__ as part of my Master's Thesis project

Master's Degree in: __Computer Science and Engineering__

Academic Year: 2023/2024

## Description

This repository contains the configuration files and launch files for performing autonomous navigation eith the AgileX Scout robot using Nav2.
It also contains the configuration files for the simulation of the robot in Ignition Gazebo v6 (Fortress).

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

## Usage

The robot can be tested in both simulated and real environments. The following steps are required to launch the robot in both environments.
Several parameters are provided in the launch files to customize the robot's behavior and the navigation stack.

### Usage in simulation environment with Gazebo

To launch the simulation of the robot in Gazebo, along with NAV2, run the following command:

```
ros2 launch agilex_scout simulate_control_gazebo.launch.py lidar_type:=<3d|2d> rviz:=<true|false>

ros2 launch scout_nav2 nav2.launch.py simulation:=true slam:=<True|False> localization:=<amcl|slam_toolbox>
```

Parameters:
- `lidar_type`: 3d for a 3D lidar (pointcloud2), 2d for a 2D lidar (laserscan)
- `rviz`: true if launching rviz, false if launching only the gazebo simulation
  

### Usage with the real robot

To launch the real scout robot with the lidar sensor and the pointcloud-to-laserscan conversion node, run the following command:

```
ros2 launch agilex_scout scout_robot_lidar.launch.py 

ros2 launch scout_nav2 nav2.launch.py simulation:=false slam:=<True|False> localization:=<amcl|slam_toolbox>
```

Parameters:

- `slam`: True if you want to use SLAM toolbox for map creation, False if you want to do localization + navigation
- `simulation`: true if running in simulation with gazebo, false if launching the real AgileX Scout robot with real sensors
- `localization`: choose localization algorithm, between `amcl` or `slam_toolbox`
