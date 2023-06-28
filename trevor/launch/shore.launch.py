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
    return LaunchDescription([
        Node(
            package='joy',
            namespace='t1',
            executable='joy_node',
            name='joy',
            parameters=[
                {"autorepeat_rate": 2.0},
                {"coalesce_interval_ms": 100}
            ]
        ),
        Node(
            package='rqt_image_view',
            namespace='t1',
            executable='rqt_image_view',
            name='rqt_image_view',
            parameters=[]
        ),
        append_launch('trevor_urdf','display.launch.py')
    ])
