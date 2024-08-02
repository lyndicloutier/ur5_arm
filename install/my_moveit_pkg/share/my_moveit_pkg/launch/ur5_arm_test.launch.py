# pylint: disable=missing-function-docstring, missing-module-docstring, line-too-long, unspecified-encoding, unused-import, missing-final-newline

# from typing import Callable
# from launch.actions import ExecuteProcess
# from moveit_configs_utils import MoveItConfigsBuilder

from __future__ import annotations
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # position_yaml = PathJoinSubstitution([FindPackageShare("my_moveit_pkg"), "config", "ur5_arm_test_goal_config.yaml"])
    position_yaml = os.path.join(get_package_share_directory('my_moveit_pkg'), 'config', 'ur5_arm_test_goal_config.yaml')

    # # prints .yaml file to screen
    # with open(position_yaml, "rt") as file:
    #     print(file.read())

    return LaunchDescription(
        [
            # Node(
            #     name = "publisher_testing",
            #     package = "my_moveit_pkg",
            #     executable = "publisher_testing",
            #     output = "screen",
            # ),
            Node(
                name = "ur5_arm_test",
                package = "my_moveit_pkg",
                executable = "ur5_arm_test",
                output = "screen",
                parameters = [position_yaml],
            ),
        ]
    )