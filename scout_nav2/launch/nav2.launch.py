import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
	slam_arg = DeclareLaunchArgument(
		name="slam",
		default_value="True",
		description="Launch SLAM or launch localization and navigation",
		choices=["True", "False"],
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
	# launch nav2 with convenient prepared launch files
	# using specialized version of nav2 bringup to account for collision monitor parameters
	nav2_bringup_dir = get_package_share_directory("nav2_bringup_custom")
	nav2_launch_file = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")

	scout_nav2_dir = get_package_share_directory("scout_nav2")
	

	# full configuration parameters file
	# choose map to load depending on test environment
	if LaunchConfiguration("simulation").perform(context) == "true":
		# Gazebo simulation
		params_file_name = "nav2_params_lidar3d.yaml"
		map_file = "warehouse/map_slam.yaml"
		use_sim_time = "true"
	elif LaunchConfiguration("simulation").perform(context) == "false":
		# Real robot
		params_file_name = "nav2_params_scout.yaml"
		map_file = "airlab/map_lidar3d_v2.yaml"
		use_sim_time = "false"


	# parameters file path
	nav2_params_file = os.path.join(scout_nav2_dir, "params", params_file_name)
	# map file path
	map_yaml_file = os.path.join(scout_nav2_dir, "maps", map_file)

	# if slam is enabled --> slam + localization + navigation
	nav2_slam_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(nav2_launch_file),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"params_file": nav2_params_file,  # full configuration parameters
			"slam": LaunchConfiguration("slam"),  # slam activated
			"map": "",  # no map,
		}.items(),
		condition=IfCondition(PythonExpression([LaunchConfiguration("slam"), " == True"])),
	)

	# if slam is disabled --> localization + navigation
	nav2_amcl_nav_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(nav2_launch_file),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"params_file": nav2_params_file,  # full configuration parameters
			"slam": LaunchConfiguration("slam"),  # localization + navigation
			"map": map_yaml_file,  # map file
		}.items(),
		condition=IfCondition(PythonExpression([LaunchConfiguration("slam"), " == False"])),
	)
	
    # static transform from world to map
	static_tf = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments=[
			"--x",
			"0.0",
			"--y",
			"0.0",
			"--z",
			"0.0",
			"--yaw",
			"0.0",
			"--pitch",
			"0.0",
			"--roll",
			"0.0",
			"--frame-id",
			"world",
			"--child-frame-id",
			"map",
		],
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

	return [nav2_slam_launch, nav2_amcl_nav_launch, rviz_node, static_tf]
