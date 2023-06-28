from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asv_drivers',
            namespace='t1',
            executable='roboteq_esc',
            name='roboteq_esc',
            parameters=[
                {"joy_topic":   'joy'},
                {"esc_port":    '/dev/ttyUSB0'}
            ]
        ),
        Node(
            package='usb_cam',
            namespace='t1',
            executable='usb_cam_node_exe',
            name='antenna_cam',
        )
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
