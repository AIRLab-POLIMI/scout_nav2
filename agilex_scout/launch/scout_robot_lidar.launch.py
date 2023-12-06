import os

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

	load_gazebo_arg = DeclareLaunchArgument(
		name="load_gazebo",
		default_value="false",
		description="load gazebo xacro file",
		choices=["true", "false"],
	)

	use_sim_time = LaunchConfiguration("use_sim_time")
	use_sim_time_arg = DeclareLaunchArgument(
		name="use_sim_time",
		default_value="false",
		description="Use simulation (Gazebo) clock if true",
		choices=["true", "false"],
	)


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
			"use_sim_time": use_sim_time,
			"base_frame": "base_link",
			"odom_frame": "odom"
		}.items()
	)

	# Scout robot description XACRO + gazebo definitions
	scout_description_file = PathJoinSubstitution([
		FindPackageShare("agilex_scout"),
		"urdf",
		"mobile_robot",
		"scout_v2.xacro",
	])
	scout_description_content = Command(
		[
			FindExecutable(name="xacro"),
			" ",
			scout_description_file,
			" load_gazebo:=",
			LaunchConfiguration("load_gazebo"),
		]
	)
	scout_description = {
		"robot_description": ParameterValue(scout_description_content, value_type=str)
	}

	scout_joint_state_publisher_node = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher_scout',
		output='screen',
		parameters=[{"use_sim_time": use_sim_time}, scout_description],
		arguments=[scout_description_file],
		remappings=[('/robot_description', '/scout_description')]
	)

	scout_robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_scout',
		output='screen',
		parameters=[{"use_sim_time": use_sim_time}, scout_description],
		arguments=[scout_description_file],
		remappings=[('/robot_description', '/scout_description')]
	)

	lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch/driver.launch.py'
            ])
        ]),
        launch_arguments={
            # for a complete list of the launch arguments, take a look at ouster_ros/launch/sensor.launch.xml
            #'sensor_hostname': "'os1-991942000600.local'",
            'viz': "false",
			'timestamp_mode': 'TIME_FROM_ROS_TIME'
        }.items()
    )

	os1_urdf = PathJoinSubstitution([
        FindPackageShare('agilex_scout'), 'urdf/sensors/os1.urdf'
	])
	os1_description_content = Command(
		[
			FindExecutable(name="xacro"),
			" ",
			os1_urdf
		]
	)
	os1_description = {'robot_description': ParameterValue(os1_description_content, value_type=str)}

	os1_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_os1',
        output='screen',
        parameters=[os1_description],
        arguments=[os1_urdf],
        remappings=[('/robot_description', '/os1_description')]
    )

	os1_urdf_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_os1',
        output='screen',
        parameters=[os1_description],
        arguments=[os1_urdf],
        remappings=[('/robot_description', '/os1_description')]
    )

	os1_tf_static = Node(
		package = "tf2_ros", 
		executable = "static_transform_publisher",
		arguments = [
			"--x", "0.15", "--y", "0.0", "--z", "0.49", 
			"--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0", 
			"--frame-id", "base_link", "--child-frame-id", "os_sensor"
		]
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
		parameters=[scout_description, os1_description],
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
		load_gazebo_arg,
		use_sim_time_arg,
		scout_base_node,
		scout_joint_state_publisher_node,
		scout_robot_state_publisher_node,
		os1_joint_state_publisher_node,
		os1_urdf_publisher_node,
		os1_tf_static,
		lidar_node,
		rviz2_node,

		#imu_tf_node,
		#imu_node
	])
