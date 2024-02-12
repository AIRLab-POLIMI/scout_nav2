## Scout NAV2: Navigation2 stack for the AgileX Scout robot

This package offers the configuration files and launch files for the navigation of the AgileX Scout robot with Nav2.

Launch files:
- `ros2 launch nav2.launch.py`: launches the entire navigation stack.

### Parking node: optimal parking pose computation

Given a target position, the parking node calculates the "optimal" goal position to be reached according to my ranking algorithm. The algorithm takes into account both the costmap (and therefore the presence of nearby obstacles), the cartesian distance from the goal, and the orientation of the robot. It is designed in such a way that the robot ends up orienting itself with the rear part facing the target, thus leaving enough maneuvering space for the arm to move towards the target position.

### Navigation stack:

The navigation stack is based on the package  `nav2_bringup` from the [Navigation2](https://navigation.ros.org/) stack. The `nav2.launch.py` launch file launches the following nodes in a composable environment (allowing faster intra-process communication):
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


### Nav2-bringup package

Note: the original nav2-bringup package has been substituted with a custom version, which is a fork of the original package. The custom version is available at [nav2-bringup-custom](../nav2_bringup/) folder. The custom version is needed to allow the use of the `nav2_collision_monitor` node, which is not available in the original package.