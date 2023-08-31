#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence

import serial
import struct

class EmAttitude:
    def __init__(self):
        self.roll = 1.1
        self.pitch = 2.2
        self.heave = 3.3
        self.heading = 4.4
    def encode(self):
        bytes = struct.pack("<BBhhhh",
                            0x90,
                            0x90,
                            int(self.roll*100),
                            int(self.pitch*100),
                            int(self.heave*100),
                            int(self.heading*100)
                            )
        print(bytes)
        return bytes



class KongsbergInterface(Node):

    def __init__(self):
        super().__init__('pilot')

        self.declare_parameter('sub/odom_topic','/t1/nav/sensors/nav/odom')
        self.declare_parameter('sub/fix_topic', '/t1/nav/sensors/nav/fix')
        self.declare_parameter('sub/nmea_topic', '/t1/nav/sensors/nmea/sentence')

        self.declare_parameter('nmea/zda_id', '$GNZDA')
        self.zda_id = self.get_parameter('nmea/zda_id').get_parameter_value().string_value

        self.declare_parameter('nmea/gga_id', '$GAGGA')
        self.gga_id = self.get_parameter('nmea/gga_id').get_parameter_value().string_value

        self.declare_parameter('serial/nmea/port', '/dev/ttyUSB0')
        self.declare_parameter('serial/nmea/baud', '9600')

        self.declare_parameter('serial/binary/port', '/dev/ttyUSB2')
        self.declare_parameter('serial/binary/baud', '19200')


        self.nmea_serial = serial.Serial(
            port=self.get_parameter('serial/nmea/port').get_parameter_value().string_value,
            baudrate=self.get_parameter('serial/nmea/baud').get_parameter_value().integer_value
        )
        self.binary_serial = serial.Serial(
            port=self.get_parameter('serial/binary/port').get_parameter_value().string_value,
            baudrate=self.get_parameter('serial/binary/baud').get_parameter_value().integer_value
        )
        print(self.binary_serial)


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

        self.nmea_sub = self.create_subscription(
            Sentence,
            self.get_parameter('sub/nmea_topic').get_parameter_value().string_value,
            self.nmea_callback,
            1)
    def odom_callback(self, odom_msg):
        self.get_logger().info('odom:')
        datagram = EmAttitude()
        self.binary_serial.write(datagram.encode())

    def fix_callback(self, fix_msg):
        pass

    def nmea_callback(self, nmea_msg):
        if nmea_msg.sentence.startswith(self.zda_id):
            self.zda_callback(nmea_msg.sentence)
        if nmea_msg.sentence.startswith(self.gga_id):
            self.gga_callback(nmea_msg.sentence)

    def zda_callback(self, zda_sentence):
        self.get_logger().info('encoding zda: %s' % zda_sentence)
        self.nmea_serial.write(zda_sentence.encode())

    def gga_callback(self, gga_sentence):
        self.get_logger().info('encoding gga: %s' % gga_sentence)
        self.nmea_serial.write(gga_sentence.encode())




def main(args=None):
    rclpy.init(args=args)

    kongsberg_interface = KongsbergInterface()

    attitude = EmAttitude()
    attitude.encode()

    rclpy.spin(kongsberg_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kongsberg_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
