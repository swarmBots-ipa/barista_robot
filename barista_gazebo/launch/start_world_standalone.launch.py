
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('barista_gazebo'), 'launch')

    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'start_world.launch.py')
        )
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gazebo_start)


    return ld
