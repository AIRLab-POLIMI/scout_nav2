## Scout NAV2: Navigation2 stack for the AgileX Scout robot

This package offers the configuration files and launch files for the navigation of the AgileX Scout robot with Nav2.
It provides a tuned set of parameters for autonomous navigation with dynamic obstacle avoidance, using the Navigation2 stack.
It also provides some maps created for both simulated and real environments in the laboratory.

#### Contributor

Author: __Simone Giampà__

Project realized at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory (AIRLAB)__ 

Project part of my Master's Thesis project for the Master's Degree in __Computer Science and Engineering__

Academic Year: 2023/2024

## Usage

The parameters for autonomous navigation will be selected automatically based on the parameters provided in the launch file.
The launch file will be needed to be modified according to the specific robot configuration and environment, 
for example for selecting a new map or changing the set of parameters for NAV2.
Launch file for the entire navigation stack:

```bash
$ ros2 launch nav2.launch.py
```

Parameters:
- `simulation`: `true` if running in simulation with gazebo, `false` if launching the real AgileX Scout robot with real sensors
- `slam`: `True` if you want to use SLAM toolbox for map creation, `False` if you want to do localization + navigation
- `localization`: choose the localization algorithm, among `amcl` and `slam_toolbox`

## Navigation stack:

The navigation stack is based on the package  `nav2_bringup` from the [Navigation2](https://navigation.ros.org/) stack. 
The `nav2.launch.py` launch file launches the following nodes in a composable environment (allowing faster intra-process communication):
- `nav2_controller_server`: the controller server node, with a goal checker, a progress checker and a MPPI controller (local planner).
- `nav2_planner_server`: the SMAC Hybrid A* node (global planner).
- `nav2_costmap_server`: the costmap server node, supporting a local costmap and a global costmap.
- `nav2_bt_navigator`: the behavior tree manager plugin.
- `nav2_behavior_server`: the behavior tree server node.
- `nav2_lifecycle_manager`: the lifecycle manager composable node.
- `nav2_map_server`: the map server node.
- `nav2_map_saver`: the map saver node.
- `nav2_amcl`: the AMCL node for localization using a laserscan topic.
- `nav2_smoother_server`: the smoother server node, for smooth trajectory computation.
- `nav2_velocity_smoother`: the velocity smoother node, for limiting velocities and accelerations.
- `nav2_collision_monitor`: the collision monitor node.
- `nav2_waypoint_follower`: the waypoint follower node.
- `slam_toolbox`: the SLAM toolbox node.
- `spatio_temporal_voxel_layer`: the spatio-temporal voxel layer node.


### Nav2-bringup package

Note: the original nav2-bringup package has been substituted with a custom version, which is a fork of the original package.
 The custom version is available at [nav2-bringup-custom](../nav2_bringup/) folder. The custom version is needed to allow 
 the use of the `nav2_collision_monitor` node, which is not available in the original package.