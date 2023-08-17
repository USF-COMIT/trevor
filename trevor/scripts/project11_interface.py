#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


from enum import Enum


class Mode(Enum):
    E_STOP = 1
    JOY = 2
    POS_HOLD = 3
    TWIS_MSG = 4


def pressed(btn_state):
    return btn_state == 1


class Project11Interface(Node):

    def __init__(self):
        super().__init__('pilot')

        self.declare_parameter('sub/odom_topic','/t1/nav/sensors/nav/odom')
        self.declare_parameter('sub/fix_topic', '/t1/nav/sensors/nav/fix')
        self.declare_parameter('sub/imu_topic', '/t1/nav/sensors/nav/filtered_imu/data')

        self.declare_parameter('pub/twist_topic', '/t1/p11_interface/twist')
        self.declare_parameter('pub/imu_topic',   '/t1/p11_interface/imu')
        self.declare_parameter('pub/fix_topic',   '/t1/p11_interface/fix')


        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('sub/odom_topic').get_parameter_value().string_value,
            self.odom_callback,
            1)

        self.fix_sub = self.create_subscription(
            NavSatFix,
            self.get_parameter('sub/fix_topic').get_parameter_value().string_value,
            self.fix_callback,
            1)

        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('sub/imu_topic').get_parameter_value().string_value,
            self.imu_callback,
            1)

        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            self.get_parameter('pub/twist_topic').get_parameter_value().string_value,
            10)

        self.fix_pub = self.create_publisher(
            NavSatFix,
            self.get_parameter('pub/fix_topic').get_parameter_value().string_value,
            10)

        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter('pub/imu_topic').get_parameter_value().string_value,
            10)

    def odom_callback(self, odom_msg):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.twist = odom_msg.twist
        twist_msg.header = odom_msg.header
        twist_msg.header.frame_id = "t1/base_link"
        self.twist_pub.publish(twist_msg)

    def fix_callback(self, fix_msg):
        pub_msg = NavSatFix()
        pub_msg = fix_msg
        pub_msg.header.frame_id = "t1/base_link"
        pub_msg.status.status = 1
        self.fix_pub.publish(pub_msg)

    def imu_callback(self, imu_msg):
        pub_msg = Imu()
        pub_msg = imu_msg
        imu_msg.header.frame_id = "t1/base_link"
        self.imu_pub.publish(pub_msg)




def main(args=None):
    rclpy.init(args=args)

    p11_interface = Project11Interface()

    rclpy.spin(p11_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    p11_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
