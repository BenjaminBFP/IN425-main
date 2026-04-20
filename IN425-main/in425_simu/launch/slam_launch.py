import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    slam_toolbox_pkg = get_package_share_directory("slam_toolbox")
    sim_pkg = get_package_share_directory("in425_simu")

    slam_params_file = LaunchConfiguration("slam_params_file", default=os.path.join(sim_pkg, "config", "slam_config.yaml"))
    
    slam_toolbox_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, "launch", "online_sync_launch.py")
        ),
        launch_arguments = {
            "slam_params_file": slam_params_file,
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_launch_file
    ])