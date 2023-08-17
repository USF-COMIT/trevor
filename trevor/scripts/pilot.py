#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from asv_interface.msg import ThrustCmd
from geometry_msgs.msg import Point
from project11_msgs.msg import Heartbeat
from project11_msgs.msg import KeyValue
from project11_msgs.msg import Helm

from enum import Enum

import math

T = ThrustCmd()


class Mode(Enum):
    E_STOP = 1
    JOY = 2
    POS_HOLD = 3
    HELM_MSG = 4


def pressed(btn_state):
    return btn_state == 1


class Pilot(Node):

    def __init__(self):
        super().__init__('pilot')

        self.declare_parameter('joy_topic', '/t1/joy')
        self.declare_parameter('joy_timeout', 1.0)
        self.declare_parameter('odom_topic', '/t1/nav/sensors/nav/odom')
        self.declare_parameter('pub/heartbeat', '/t1/project11/status/helm')
        self.declare_parameter('sub/helm', '/t1/project11/control/helm')

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

        self.helm_sub = self.create_subscription(
            Helm,
            self.get_parameter('sub/helm').get_parameter_value().string_value,
            self.helm_callback,
            1)

        self.thrust_pub = self.create_publisher(ThrustCmd, '/t1/thrust_cmd', 10)

        self.heart_pub = self.create_publisher(
            Heartbeat,
            self.get_parameter('pub/heartbeat').get_parameter_value().string_value,
            10
        )

        self.thuster_gain = 1.0
        self.mode = Mode.JOY

        self.loop_period = 0.01
        self.timer = self.create_timer(self.loop_period, self.control_loop)

        self.last_odom = Odometry()
        self.hold_pos = Point()

        self.rudder_gain = 1.0
        self.reverse_gain = 1.25
        self.throttle_gain = 0.8

    def control_loop(self):
        self.send_hb()
        # self.pos_hold()

    def send_hb(self):
        kv = KeyValue()
        kv.key = "trevor_ready"
        kv.value = "true"

        hb = Heartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        self.heart_pub.publish(hb)

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

        if joy_cmd_age_sec > self.joy_timeout or joy_cmd_age_sec < -1:
            self.get_logger().warn('Joy MSG to old! ignoring: %s' % joy_cmd_age_sec)
            #self.set_mode(Mode.POS_HOLD)

        if pressed(msg.buttons[0]):
            self.set_mode(Mode.JOY)
        if pressed(msg.buttons[2]):
            self.set_mode(Mode.HELM_MSG)
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
        # self.pos_hold()

    def joy_control(self, joy_msg):
        if self.mode == Mode.JOY:
            cmd_msg = ThrustCmd()
            cmd_msg.cmd = [joy_msg.axes[1], joy_msg.axes[4]]
            cmd_msg.header.stamp = self.get_clock().now().to_msg()

            self.send_thrust_cmd(cmd_msg)

    def helm_callback(self, helm_msg):
        if self.mode == Mode.HELM_MSG:
            rudder = helm_msg.rudder
            throttle = helm_msg.throttle * self.throttle_gain
            #self.get_logger().warn('rudder,throttle : %s, %s' % (rudder, throttle))
            port = rudder * self.rudder_gain
            stbd = -rudder * self.rudder_gain

            #self.get_logger().warn('raw thrust: %s, %s' % (port, stbd))

            if port < 0:
                port *= self.reverse_gain
            if stbd < 0:
                stbd *= self.reverse_gain

            #self.get_logger().warn('rev_gain thrust: %s, %s' % (port, stbd))
            # if port > 1:
            #     sub = port - 1
            #     port -= sub
            #     stbd -= sub
            # if stbd > 1:
            #     sub = stbd - 1
            #     port -= sub
            #     stbd -= sub
            # if port < -1:
            #     sub = port + 1
            #     port -= sub
            #     stbd -= sub
            # if stbd < -1:
            #     sub = stbd + 1
            #     port -= sub
            #     stbd -= sub

            if min(port, stbd) < -1:
                normilzation_factor = 1/abs(min(port, stbd))

                port *= normilzation_factor
                stbd *= normilzation_factor

            #self.get_logger().warn('clamped thrust: %s, %s' % (port, stbd))

            remaining_power_fwd = 1 - max(port, stbd)
            remaining_power_rev = 1 + min(port, stbd)
            remaining_power = min(remaining_power_fwd,remaining_power_rev)
            #self.get_logger().warn('remaining power fwd/rev: %s %s' % (remaining_power_fwd,remaining_power_rev))
            #self.get_logger().warn('throttle: %s' % throttle)
            if throttle > 0:
                throttle = min(throttle,  remaining_power)
                port += throttle
                stbd += throttle
            else:
                throttle = max(throttle, remaining_power_rev)
                port += throttle
                stbd += throttle
            #self.get_logger().warn('throttle clamped: %s' % throttle)
            #self.get_logger().warn('thrust: %s, %s' % (port, stbd))



            cmd_msg = ThrustCmd()
            cmd_msg.cmd = [port, stbd]
            cmd_msg.header.stamp = self.get_clock().now().to_msg()

            self.send_thrust_cmd(cmd_msg)


    # def transform_pose(input_pose, from_frame, to_frame):
    #
    #     # **Assuming /tf2 topic is being broadcasted
    #     tf_buffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tf_buffer)
    #
    #     pose_stamped = tf2_geometry_msgs.PoseStamped()
    #     pose_stamped.pose = input_pose
    #     pose_stamped.header.frame_id = from_frame
    #     pose_stamped.header.stamp = rospy.Time.now()
    #
    #     try:
    #         # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
    #         output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
    #         return output_pose_stamped.pose
    #
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         raise
    # def pos_hold(self):
    #     if self.mode == Mode.POS_HOLD:
    #         orientation_q = self.last_odom.pose.pose.orientation
    #         orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #         (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #
    #         vessel_position = self.last_odom.pose.pose.position
    #         vect_to_target = Point()
    #         vect_to_target.x = self.hold_pos.x - vessel_position.x
    #         vect_to_target.y = self.hold_pos.y - vessel_position.y


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
