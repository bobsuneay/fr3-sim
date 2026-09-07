"""Offline mock: MoveIt Plan and Execute affect only the virtual robot."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_real_bringup'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/bringup.launch.py')),
        launch_arguments={'mode': 'mock', 'enable_execution': 'true'}.items())])
