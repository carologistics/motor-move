# Copyright (c) 2026 Carologistics
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
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Use source YAML directly so changes take effect without rebuild
    package_dir = get_package_share_directory("motor_move")
    source_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "motor_move.yaml",
    )
    install_config = os.path.join(package_dir, "config", "motor_move.yaml")
    config = source_config if os.path.exists(source_config) else install_config

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="/robotinobase1", description="Namespace for the MotorMove node."
    )

    # Node definition
    motor_move_node = Node(
        package="motor_move",
        executable="motor_move",
        namespace=namespace,
        name="motor_move",
        output="screen",
        parameters=[config, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            namespace_arg,
            motor_move_node,
        ]
    )
