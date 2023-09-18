from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def append_launch(package, launch):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package), 'launch'),
            '/'+launch])
        )


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('trevor'),
        'config',
        'trevor.yaml'
        )

    return LaunchDescription([
        Node(
            package='trevor',
            namespace='t1',
            executable='kongsberg_interface.py',
            name='kongsberg_interface',
            parameters=[config]
        ),
    ])
