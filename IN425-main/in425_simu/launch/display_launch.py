import os, xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("in425_desc"), "xacro", "robot.xacro")

    return LaunchDescription([
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            parameters = [
                {"robot_description": xacro.process_file(xacro_file, mappings={"base_color_arg": "White"}).toxml()}
            ]
        )
    ])