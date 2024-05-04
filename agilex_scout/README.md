# AgileX Scout ROS2 package for URDF XACRO descriptions, Gazebo simulation, and real robot control

This package offers the configuration files and launch files for the AgileX Scout robot. It also contains the configuration
files for the simulation of the robot in Gazebo, in the simulated environment of the AWS RoboMaker Small Warehouse World.

#### Contributor

Author: __Simone Giampà__

Project realized at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory (AIRLAB)__ 

Project part of my Master's Thesis project for the Master's Degree in __Computer Science and Engineering__

Academic Year: 2023/2024

## Dependencies

The following dependencies are required to run the navigation stack with the Scout robot:

- [ouster ros2 driver](https://github.com/ouster-lidar/ouster-ros/tree/ros2): Official ROS driver for Ouster sensors
- [scout base package](https://github.com/agilexrobotics/scout_ros2): minimal packages to control the scout robot using a ROS2 wrapper for the UGV SDK.
- [UGV SDK](https://github.com/westonrobot/ugv_sdk): C++ interface to communicate with the mobile platforms, 
  for sending commands to the robot and receiving the latest robot state.
- [pointcloud to laserscan ros2](https://github.com/ros-perception/pointcloud_to_laserscan): conversion from pointcloud2 data to laserscan data,
  using volumetric projection on a virtual plane. This package is needed in simulation as well.

## Usage

The simulation environment is compatible with Ignition Gazebo v6 (Fortress). The robot is a skid-steering mobile robot with a 3D lidar
sensor mounted on top.

Two launch files are available for the simulation of the robot in Gazebo:
- `simulate_control_gazebo.launch.py`: launch the robot with the lidar sensor, either with 3d pointcloud or with 2d laserscan, 
  and the RViz GUI (optional). It launches also `pointcloud-to-laserscan` node to convert the pointcloud2 data to laserscan data.
- `scout_robot_lidar.launch.py`: launch the real robot with the lidar Ouster OS1, and the laserscan conversion node. 
  It launches also the `scout_base` node to control the robot and receive the odometry data.

To launch the simulated robot in Gazebo, run the following command:

```bash
$ ros2 launch scout_gazebo simulate_control_gazebo.launch.py lidar:=<2d|3d> rviz:=<true|false>
```

To launch the real robot with the Ouster OS1 lidar, run the following command:

```bash
$ ros2 launch scout_gazebo scout_robot_lidar.launch.py
```

## Usage in conjunction with the Igus Rebel robot arm

This package does not contain the URDf description for the collision boundaries required when mounting the Igus Rebel robot arm 
on the AgileX Scout robot. This URDF description is available in the `igus_rebel_description_ros2` package.
In order to start up the Igus Rebel robot arm with the AgileX Scout robot, refer to the other repository:
[ros2-igus-rebel](https://github.com/AIRLab-POLIMI/ros2-igus-rebel)
