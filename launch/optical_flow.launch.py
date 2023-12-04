# Copyright 2023 University of Leeds.
# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    optical_flow_params_path = PathJoinSubstitution(
        [FindPackageShare("optical_flow_ros"), "config", "sensor_params.yaml"]
    )

    optical_flow_node = Node(
        package="optical_flow_ros",
        executable="optical_flow_node",
        # name="optical_flow",
        # Do not change, else config params and remappings need to be updated
        namespace="",
        output="screen",
        parameters=[optical_flow_params_path],
        # remappings=[('odom', 'flow_odom')]
    )

    ld = LaunchDescription()
    ld.add_action(optical_flow_node)
    return ld
