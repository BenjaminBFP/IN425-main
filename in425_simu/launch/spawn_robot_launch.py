import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simu_pkg = get_package_share_directory("in425_simu")

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, "launch", "display_launch.py")
        )
    )

    robot_pose = {"x": -2.0, "y": 0.0, "yaw": 0.0}

    return LaunchDescription([
        display_launch,

        Node(
            package = "gazebo_ros",
            executable = "spawn_entity.py",
            arguments = [
                "-entity", "robot_ipsa", 
                "-topic", "/robot_description",
                "-x", str(robot_pose["x"]), 
                "-y", str(robot_pose["y"]),
                "-Y", str(robot_pose["yaw"]),
            ]
        )
    ])