import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    simu_pkg = get_package_share_directory("in425_simu")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        )
    )
    os.environ["GAZEBO_MODEL_PATH"] += os.path.join(simu_pkg, "models")

    stop_previous_gazebo = ExecuteProcess(cmd=[
        "killall",
        "gzserver",
        "gzclient"
    ], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value = os.path.join(simu_pkg, "worlds", "env.world"),
            description = "World file to use for the simulation"
        ),

        stop_previous_gazebo,

        RegisterEventHandler(
            event_handler = OnProcessExit(
              target_action = stop_previous_gazebo,
              on_exit = [gazebo_launch]  #start this node when previous gazebo processes have been stopped
            )
        ),
    ])