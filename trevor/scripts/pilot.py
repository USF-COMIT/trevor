#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node


from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from asv_interface.msg import ThrustCmd
from geometry_msgs.msg import Point

from enum import Enum

import math

T = ThrustCmd()


class Mode(Enum):
    E_STOP = 1
    JOY = 2
    POS_HOLD = 3
    TWIS_MSG = 4


def pressed(btn_state):
    return btn_state == 1


class Pilot(Node):

    def __init__(self):
        super().__init__('pilot')

        self.declare_parameter('joy_topic', '/t1/joy')
        self.declare_parameter('joy_timeout', 1.0)
        self.declare_parameter('odom_topic', '/t1/nav/sensors/nav/odom')

        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.joy_timeout = self.get_parameter('joy_timeout').get_parameter_value().double_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            1)


        self.thrust_pub = self.create_publisher(ThrustCmd, '/t1/thrust_cmd', 10)


        self.thuster_gain = 1.0
        self.mode = Mode.JOY

        self.loop_period = 0.01
        self.timer = self.create_timer(self.loop_period, self.control_loop)

        self.last_odom = Odometry()
        self.hold_pos = Point()

    def control_loop(self):
        self.pos_hold()

    def set_mode(self, mode):
        self.mode = mode
        self.get_logger().info('Mode Set: %s' % mode)
        if self.mode == Mode.POS_HOLD:
            self.hold_pos = self.last_odom.pose.pose.position

    def set_thruster_gain(self, gain):
        self.thuster_gain = gain
        self.get_logger().info('Thruster Gain Set: %s' % gain)

    def send_thrust_cmd(self, cmd_msg):
        cmd_msg.cmd[0] *= self.thuster_gain
        cmd_msg.cmd[1] *= self.thuster_gain
        self.thrust_pub.publish(cmd_msg)
        # self.get_logger().info('ThrustCmd: %s' % cmd_msg)

    def joy_callback(self, msg):
        current_time = self.get_clock().now()
        rx_time = Time.from_msg(msg.header.stamp)
        joy_cmd_age = current_time - rx_time
        joy_cmd_age_sec = joy_cmd_age.nanoseconds / 1e+9

        if joy_cmd_age_sec > self.joy_timeout or joy_cmd_age_sec < 0:
            self.get_logger().warn('Joy MSG to old! Holding Postion: %s' % joy_cmd_age_sec)
            self.set_mode(Mode.POS_HOLD)

        if pressed(msg.buttons[0]):
            self.set_mode(Mode.JOY)
        if pressed(msg.buttons[2]):
            self.set_mode(Mode.POS_HOLD)
        if pressed(msg.buttons[1]):
            self.set_mode(Mode.E_STOP)
        if pressed(msg.buttons[1]):
            self.set_mode(Mode.E_STOP)

        if msg.axes[7] == 1:
            self.set_thruster_gain(1.0)
        if msg.axes[6] == 1:
            self.set_thruster_gain(0.60)
        if msg.axes[7] == -1:
            self.set_thruster_gain(0.25)

        # self.get_logger().info('JoyMsg: %s' % msg)
        self.joy_control(msg)

    def odom_callback(self, odom_msg):
        self.last_odom = odom_msg
        self.pos_hold()
    def joy_control(self, joy_msg):
        if self.mode == Mode.JOY:
            cmd_msg = ThrustCmd()
            cmd_msg.cmd = [joy_msg.axes[1] , joy_msg.axes[4] ]
            cmd_msg.header.stamp = self.get_clock().now().to_msg()

            self.send_thrust_cmd(cmd_msg)

    def transform_pose(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
    def pos_hold(self):
        if self.mode == Mode.POS_HOLD:
            orientation_q = self.last_odom.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            vessel_position = self.last_odom.pose.pose.position
            vect_to_target = Point()
            vect_to_target.x = self.hold_pos.x - vessel_position.x
            vect_to_target.y = self.hold_pos.y - vessel_position.y








def main(args=None):
    rclpy.init(args=args)

    pilot = Pilot()

    rclpy.spin(pilot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
