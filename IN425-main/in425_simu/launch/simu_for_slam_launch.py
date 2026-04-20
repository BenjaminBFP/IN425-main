import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    simu_pkg = get_package_share_directory("in425_simu")

    start_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, "launch", "start_world_launch.py")
        )
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, "launch", "spawn_robot_launch.py")
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, "launch", "slam_launch.py")
        )
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ['-d', os.path.join(simu_pkg, "rviz", "mapping.rviz")]
    )

    return LaunchDescription([
        start_world_launch,
        TimerAction(
           period = 5.0,
           actions = [
                spawn_robot_launch
           ]
        ),
        slam_launch,
        rviz_node
    ])