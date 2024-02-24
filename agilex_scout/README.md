# AgileX Scout ROS2 package for URDF description, Gazebo simulation, and real robot control

## AgileX Scout robot with ROS2 and Ignition Gazebo Fortress v6

This package offers the configuration files and launch files for the AgileX Scout robot. It also contains the configuration files for the simulation of the robot in Gazebo, in the simulated environment of the AWS RoboMaker Small Warehouse World.

Two launch files are available for the simulation of the robot in Gazebo:
- `simulate_control_gazebo.launch.py`: launch the robot with the lidar sensor, either with 3d pointcloud or with 2d laserscan, and the RViz GUI (optional). It launches also `pointcloud_to_laserscan` node to convert the pointcloud2 data to laserscan data.
- `scout_robot_lidar.launch.py`: launch the real robot with the lidar Ouster OS1, and the laserscan conversion node. It launches also the `scout_base` node to control the robot and receive the odometry data.

## Required dependencies for real scout rebot:

- [ouster ros2 driver](https://github.com/ouster-lidar/ouster-ros/tree/ros2): Official ROS driver for Ouster sensors
- [scout base package](https://github.com/agilexrobotics/scout_ros2): minimal packages to control the scout robot using a ROS2 wrapper for the UGV SDK.
- [UGV SDK](https://github.com/westonrobot/ugv_sdk): C++ interface to communicate with the mobile platforms, for sending commands to the robot and receiving the latest robot state.
- [pointcloud to laserscan ros2](https://github.com/ros-perception/pointcloud_to_laserscan): conversion from pointcloud2 data to laserscan data, using volumetric projection on a virtual plane. This package is needed in simulation as well.