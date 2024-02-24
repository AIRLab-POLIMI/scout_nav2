from math import pi

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
	Command,
	FindExecutable,
	LaunchConfiguration,
	PythonExpression,
)


def generate_launch_description():

	# scout base ros2 node launcher
	scout_base_node = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			PathJoinSubstitution([
				FindPackageShare('scout_base'),
				'launch/scout_base.launch.py'
			])
		]),
		launch_arguments={
			"odom_topic_name": "/odometry",
			"use_sim_time": "false",
			"base_frame": "base_footprint",
			"odom_frame": "odom"
		}.items()
	)

	# Scout robot description XACRO + gazebo definitions
	scout_description_file = PathJoinSubstitution([
		FindPackageShare("agilex_scout"),
		"urdf",
		"robot.urdf.xacro",
	])
	scout_description_content = Command(
		[
			FindExecutable(name="xacro"),
			" ",
			scout_description_file,
			" load_gazebo:=false",
			" odometry_source:=_", # not used
			" simulation:=false"
		]
	)
	scout_description = {
		"robot_description": ParameterValue(scout_description_content, value_type=str)
	}

	scout_robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_scout',
		output='screen',
		parameters=[{"use_sim_time": False}, scout_description],
		arguments=[scout_description_file],
		remappings=[('/robot_description', '/scout/robot_description')]
	)


	lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch/driver.launch.py'
            ])
        ]),
        launch_arguments={
            # config/driver_params.yaml is the file containing the list of parameters used by the driver
            'viz': "false",
        }.items()
    )
	
	pointcloud_to_laserscan_node = Node(
		package='pointcloud_to_laserscan',
		executable='pointcloud_to_laserscan_node',
		name='pointcloud_to_laserscan_node',
		remappings=[('cloud_in', "/ouster/points"),
					('scan', "/ouster/scan")],
		parameters=[{
			'transform_tolerance': 0.05,
			'min_height': -0.64,
			'max_height': 1.0,
			'angle_min': -pi,
			'angle_max': pi,
			'angle_increment': pi / 180.0 / 2.0,
			'scan_time': 1/10.0, # 10Hz
			'range_min': 0.1,
			'range_max': 100.0,
			'use_inf': True,
		}],
	)

	# rviz2 node
	rviz2_file = PathJoinSubstitution([
		FindPackageShare("agilex_scout"),
		"rviz",
		"real_scout_display.rviz",
	])

	rviz2_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz2_file],
		parameters=[scout_description],
	)

	# IMU launch 
	#imu_tf_node = Node(
			#package = "tf2_ros",
			#executable = "static_transform_publisher",
			##arguments = ["-0.158", "-0.388", "0.012", "-0.1521667", "-0.0594674", "-0.1479408", "0.9754089", "os_lidar", "sensor"]
			#arguments = ["0.15", "0.0", "-0.4", "0.0112575", "-0.0050925", "0.9999026", "-0.0064842", "os_lidar", "sensor"]
	#)

	#imu_node = IncludeLaunchDescription(
		#PythonLaunchDescriptionSource([
			#PathJoinSubstitution([
				#FindPackageShare('microstrain_inertial_driver'),
				#'launch/microstrain_launch.py'
			#])
		#]),
		#launch_arguments={
			#'params_file': '/home/airlab/imu_params_1.yml',
			#'configure': 'true',
			#'activate': 'true'
		#}.items()
	#)

	#complementary_filter_node = GroupAction(
		#actions=[
			#SetRemap(src='/imu/data_raw', dst='/ouster/imu'),
			#LaunchDescription([
				#Node(
					#package='imu_complementary_filter',
					#executable='complementary_filter_node',
					#name='complementary_filter_node'
				#)
			#])
		#]
	#)

	return LaunchDescription([
		scout_base_node,
		scout_robot_state_publisher_node,
		lidar_node,
		rviz2_node,
		pointcloud_to_laserscan_node,

		#imu_tf_node,
		#imu_node
	])
