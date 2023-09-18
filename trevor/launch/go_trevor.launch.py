from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def append_launch(package, launch):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package), 'launch'),
            '/'+launch])
        )



def generate_launch_description():
    return LaunchDescription([
        append_launch('trevor', 'core.launch.py'),
        append_launch('trevor', 'microstrain.launch.py'),
        append_launch('trevor', 'ntrip.launch.py'),
        append_launch('trevor', 'kongsberg_interface.py')
    ])
