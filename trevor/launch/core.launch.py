from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('trevor'),
        'config',
        'trevor.yaml'
        )

    return LaunchDescription([
        Node(
            package='asv_drivers',
            namespace='t1',
            executable='roboteq_esc',
            name='roboteq_esc',
            respawn=True,
            respawn_delay=4,
            parameters=[config]
        ),
#        Node(
#            package='usb_cam',
#            namespace='t1',
#            executable='usb_cam_node_exe',
#            name='antenna_cam',
#        ),
        Node(
            package='trevor',
            namespace='t1',
            executable='pilot.py',
            respawn=True,
            respawn_delay=4,
            name='pilot',
        ),
        Node(
            package='trevor',
            namespace='t1',
            executable='project11_interface.py',
            respawn=True,
            respawn_delay=4,
            name='project11_interface',
        )


        # Node(
        #     package='usb_cam',
        #     namespace='t1',
        #     executable='usb_cam_node_exe',
        #     name='antenna_cam',
        # ),

        #,
#        Node(
#            package='joy',
#            namespace='t1',
#            executable='joy_node',
#            name='joy',
#            parameters=[
#                {"autorepeat_rate":   2.0},
#                {"coalesce_interval_ms":   100}
#            ]
#        ),
    ])
