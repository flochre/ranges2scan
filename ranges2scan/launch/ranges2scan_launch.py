# Copyright 2023 flochre
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

ranges2scan_params = PathJoinSubstitution([
    FindPackageShare("ranges2scan"),
    "params",
    "ranges2scan_params.yaml",
])

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=ranges2scan_params,
            description='Full path to the ROS2 parameters file to be used by ranges2scan node'),

        Node(
            package='ranges2scan',
            executable='ranges2scan_node',
            name='ranges2scan',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
