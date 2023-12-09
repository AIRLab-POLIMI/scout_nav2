import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
	slam_arg = DeclareLaunchArgument(
		name="slam",
		default_value="true",
		description="Launch SLAM or launch localization and navigation",
		choices=["true", "false"],
	)

	simulation_arg = DeclareLaunchArgument(
		name="simulation",
		default_value="true",
		description="Launch simulation with gazebo or launch real robot navigation",
		choices=["true", "false"],
	)

	return LaunchDescription(
		[
			slam_arg,
			simulation_arg,
			OpaqueFunction(function=launch_setup),
		]
	)


def launch_setup(context, *args, **kwargs):
	# launch nav2 with convenient prepared launch file
	nav2_bringup_dir = get_package_share_directory("nav2_bringup")
	nav2_launch_file = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")

	scout_nav2_dir = get_package_share_directory("scout_nav2")

	# map file
	map_yaml_file = os.path.join(
		scout_nav2_dir,
		"maps",
		"map_slam.yaml",
	)

	# full configuration parameters file
	if LaunchConfiguration("simulation").perform(context):
		params_file_name = "nav2_params.yaml"
	else:
		params_file_name = "nav2_params_scout.yaml"
	nav2_params_file = os.path.join(scout_nav2_dir, "params", params_file_name)

	use_sim_time = (
		"true" if LaunchConfiguration("simulation").perform(context) else "false"
	)

	# if slam is enabled --> slam + localization + navigation
	nav2_slam_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(nav2_launch_file),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"params_file": nav2_params_file,  # full configuration parameters
			"slam": "True",  # slam activated
			"map": "",  # no map
		}.items(),
		condition=IfCondition(LaunchConfiguration("slam")),
	)

	# if slam is disabled --> localization + navigation
	nav2_amcl_nav_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(nav2_launch_file),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"params_file": nav2_params_file,  # full configuration parameters
			"slam": "False",  # localization + navigation
			"map": map_yaml_file,  # map file
		}.items(),
		condition=UnlessCondition(LaunchConfiguration("slam")),
	)

	# launch rviz
	rviz_default_config_file = os.path.join(scout_nav2_dir, "rviz", "nav2.rviz")
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		arguments=["-d", rviz_default_config_file],
	)

	return [nav2_slam_launch, nav2_amcl_nav_launch, rviz_node]
