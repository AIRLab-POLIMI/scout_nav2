# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from os import environ
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, TimerAction


def generate_launch_description():
    # Get the launch directory
    aws_small_warehouse_dir = get_package_share_directory(
        "aws_robomaker_small_warehouse_world"
    )

    # Launch configuration variables specific to simulation
    world = LaunchConfiguration("world")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    world_sdf_file = os.path.join(
        aws_small_warehouse_dir, "worlds", "small_warehouse", "small_warehouse.world"
    )

    world_file_arg = DeclareLaunchArgument(
        name="world",
        default_value=world_sdf_file,
        description="Full path to world model file to load",
    )

    # Gazebo GUI configuration file
    gazebo_config_gui_file = os.path.join(
        aws_small_warehouse_dir,
        "gui",
        "gazebo_gui.config",
    )

    # Ignition Gazebo 6 environment variables (local to the running instance)
    env = {  # IGN GAZEBO FORTRESS env variables
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ":".join(
            [
                environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
            ]
        ),
    }

    # Setup to launch the simulator and Gazebo world
    gazebo_sim_process = ExecuteProcess(
        cmd=[
            "ign gazebo",
            "--verbose 1 -r --gui-config " + gazebo_config_gui_file,
            world,
        ],
        output="log",
        additional_env=env,
        shell=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([use_sim_time_arg, world_file_arg, gazebo_sim_process])
