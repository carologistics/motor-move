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
from launch.substitutions import LaunchConfiguration  # noqa: F401
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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
    enable_tuning_log = LaunchConfiguration("enable_tuning_log")
    enable_live_tuning = LaunchConfiguration("enable_live_tuning")
    tuning_remote_target = LaunchConfiguration("tuning_remote_target")

    # Parameter substitutions dictionary
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "enable_tuning_log": enable_tuning_log,
        "enable_live_tuning": enable_live_tuning,
        "tuning_remote_target": tuning_remote_target,
    }

    # Create temporary YAML with substitutions
    # Uses "motor_move" as root key - same config for all robots
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config,
            root_key="motor_move",  # Loads motor_move/ros__parameters (same for all robots)
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="/", description="Namespace for the MotorMove node."  # default to the root namespace
    )

    enable_tuning_log_arg = DeclareLaunchArgument(
        "enable_tuning_log", default_value="false", description="Enable CSV logging of PID data for tuning analysis"
    )

    enable_live_tuning_arg = DeclareLaunchArgument(
        "enable_live_tuning", default_value="false", description="Enable live PID parameter tuning via ros2 param set"
    )

    tuning_remote_target_arg = DeclareLaunchArgument(
        "tuning_remote_target",
        default_value="",
        description="Remote SCP target for auto-transfer (e.g. sam@192.168.1.100:/home/sam/ros2/pid_tuning)",
    )

    # Node definition
    motor_move_node = Node(
        package="motor_move",
        executable="motor_move",
        namespace=namespace,
        name="motor_move",
        output="screen",
        parameters=[configured_params],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            namespace_arg,
            enable_tuning_log_arg,
            enable_live_tuning_arg,
            tuning_remote_target_arg,
            motor_move_node,
        ]
    )
