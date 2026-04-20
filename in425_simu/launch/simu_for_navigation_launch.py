import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
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

    map_server_node = Node(
        package = "nav2_map_server",
        executable = "map_server",
        parameters = [{"yaml_filename" :os.path.join(simu_pkg, "maps", "my_map.yaml")}],
        output = "screen"
    )

    activate_map_server_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd = ['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
                output="screen")
        ]
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ['-d', os.path.join(simu_pkg, "rviz", "config.rviz")]
    )

    #publish static transformation btw map and odom
    static_tf_publisher = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        start_world_launch,
        TimerAction(
           period = 5.0,
           actions = [
                spawn_robot_launch
           ]
        ),
        map_server_node,
        activate_map_server_cmd,
        rviz_node,
        static_tf_publisher
    ])