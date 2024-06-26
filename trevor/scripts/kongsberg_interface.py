#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
import socket
import datetime

from euler_from_quaternion import *

import serial
import struct

import pynmea2


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
                            int(self.roll * 100),
                            int(self.pitch * 100),
                            int(self.heave * 100),
                            int(self.heading * 100)
                            )
        # print(bytes)
        return bytes


class KmBinary:
    def __init__(self):
        self.start_id = "#KMB"
        self.datagram_length = 132
        self.datagram_version = 1
        self.utc_seconds = 0
        self.utc_nanoseconds = 0
        self.status_word = 0
        self.latitude = 0
        self.longitude = 0
        self.ellipsoid_height = 0
        self.roll = 0
        self.pitch = 0
        self.heading = 0
        self.heave = 0
        self.roll_rate = 0
        self.pitch_rate = 0
        self.yaw_rate = 0
        self.north_velocity = 0
        self.east_velocity = 0
        self.down_velocity = 0
        self.latitude_error = 0
        self.longitude_error = 0
        self.height_error = 0
        self.roll_error = 0
        self.pitch_error = 0
        self.heading_error = 0
        self.heave_error = 0
        self.north_acceleration = 0
        self.east_acceleration = 0
        self.down_acceleration = 0
        # delayed heave:
        self.dh_utc_seconds = 0
        self.dh_utc_nanoseconds = 0
        self.delayed_heave = 0

    def encode(self):
        # <4sHHLLLddfffffffffffffffffffffLLL
        datagram = struct.pack("<4s2H3I2d21fIIf",
                               bytes(self.start_id, 'utf-8'),
                               self.datagram_length,
                               self.datagram_version,
                               self.utc_seconds,
                               self.utc_nanoseconds,
                               self.status_word,
                               self.latitude,
                               self.longitude,
                               self.ellipsoid_height,
                               self.roll,
                               self.pitch,
                               self.heading,
                               self.heave,
                               self.roll_rate,
                               self.pitch_rate,
                               self.yaw_rate,
                               self.north_velocity,
                               self.east_velocity,
                               self.down_velocity,
                               self.latitude_error,
                               self.longitude_error,
                               self.height_error,
                               self.roll_error,
                               self.pitch_error,
                               self.heading_error,
                               self.heave_error,
                               self.north_acceleration,
                               self.east_acceleration,
                               self.down_acceleration,
                               # delayed heave:
                               self.dh_utc_seconds,
                               self.dh_utc_nanoseconds,
                               self.delayed_heave
                               )
        # print(datagram)
        return datagram


def set_bit(value, bit):
    return value | (1 << bit)


class KongsbergInterface(Node):

    def __init__(self):
        super().__init__('pilot')

        self.declare_parameter('sub.odom_topic', '/t1/nav/sensors/nav/odom')
        self.declare_parameter('sub.fix_topic', '/t1/nav/sensors/nav/fix')
        self.declare_parameter('sub.nmea_topic', '/t1/nav/sensors/nmea/sentence')

        self.declare_parameter('nmea.zda_id', '$GNZDA')
        self.zda_id = self.get_parameter('nmea.zda_id').get_parameter_value().string_value

        self.declare_parameter('nmea.gga_id', '$--GGA')  #$GAGGA
        self.gga_id = self.get_parameter('nmea.gga_id').get_parameter_value().string_value

        self.declare_parameter('serial.nmea.port', '/dev/ttyS1')
        self.declare_parameter('serial.nmea.baud', 115200)

#        self.declare_parameter('serial.binary/port', '/dev/ttyUSB2')
#        self.declare_parameter('serial.binary/baud', 19200)

        self.declare_parameter('udp.ip', '192.168.1.102')
        self.declare_parameter('udp.port', 3000)
        self.udp_ip = self.get_parameter('udp.ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp.port').get_parameter_value().integer_value

        baud = self.get_parameter('serial.nmea.baud').get_parameter_value().integer_value
        self.get_logger().info('Opening Port: %s baud: %s' % (self.get_parameter('serial.nmea.port').get_parameter_value().string_value, baud))
        try:
            self.nmea_serial = serial.Serial(
                port=self.get_parameter('serial.nmea.port').get_parameter_value().string_value,
                baudrate=self.get_parameter('serial.nmea.baud').get_parameter_value().integer_value
            )
        except Exception as ex:
            self.get_logger().error('An error occurred trying to open the serial port: \n%s' % (str(ex)) )
            raise SystemExit


#        self.binary_serial = serial.Serial(
#            port=self.get_parameter('serial.binary/port').get_parameter_value().string_value,
#            baudrate=self.get_parameter('serial.binary/baud').get_parameter_value().integer_value
#        )
        self.sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_DGRAM)  # UDP

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('sub.odom_topic').get_parameter_value().string_value,
            self.odom_callback,
            1)

        self.fix_sub = self.create_subscription(
            NavSatFix,
            self.get_parameter('sub.fix_topic').get_parameter_value().string_value,
            self.fix_callback,
            1)

        self.nmea_sub = self.create_subscription(
            Sentence,
            self.get_parameter('sub.nmea_topic').get_parameter_value().string_value,
            self.nmea_callback,
            1)

    def odom_callback(self, odom_msg):
        # self.get_logger().info('odom:')
        (roll, pitch, yaw) = euler_from_quaternion(odom_msg.pose.pose.orientation.x,
                                                   odom_msg.pose.pose.orientation.y,
                                                   odom_msg.pose.pose.orientation.z,
                                                   odom_msg.pose.pose.orientation.w)

        roll *= 180 / math.pi
        pitch *= - 180 / math.pi
        yaw *= 180 / math.pi

        (roll_ned, pitch_ned, hdg) = euler_from_quaternion(odom_msg.pose.pose.orientation.y,
                                                           odom_msg.pose.pose.orientation.x,
                                                           -odom_msg.pose.pose.orientation.z,
                                                           odom_msg.pose.pose.orientation.w)

        hdg *= 180 / math.pi
        hdg = hdg + 90
        while hdg < 0:
            hdg += 360

        datagram = KmBinary()
        datagram.utc_seconds = odom_msg.header.stamp.sec
        datagram.utc_nanoseconds = odom_msg.header.stamp.nanosec
        datagram.status_word = 0b10011100000000000000000000000000
        datagram.latitude = odom_msg.pose.pose.position.x
        datagram.longitude = odom_msg.pose.pose.position.y
        datagram.ellipsoid_height = odom_msg.pose.pose.position.z
        datagram.roll = roll
        datagram.pitch = pitch
        datagram.heading = hdg
        datagram.heave = 0
        datagram.roll_rate = odom_msg.twist.twist.angular.x * 180 / math.pi
        datagram.pitch_rate = -odom_msg.twist.twist.angular.y * 180 / math.pi
        datagram.yaw_rate = -odom_msg.twist.twist.angular.z * 180 / math.pi
        datagram.north_velocity = 0
        datagram.east_velocity = 0
        datagram.down_velocity = 0
        datagram.latitude_error = -1
        datagram.longitude_error = -1
        datagram.height_error = -1
        datagram.roll_error = -1
        datagram.pitch_error = -1
        datagram.heading_error = -1
        datagram.heave_error = -1
        datagram.north_acceleration = 0
        datagram.east_acceleration = 0
        datagram.down_acceleration = 0
        # delayed heave:
        datagram.dh_utc_seconds = odom_msg.header.stamp.sec
        datagram.dh_utc_nanoseconds = odom_msg.header.stamp.nanosec
        datagram.delayed_heave = 0
        # self.binary_serial.write(datagram.encode())
        byte_array = datagram.encode()
        self.sock.sendto(byte_array, (self.udp_ip, self.udp_port))

    def fix_callback(self, fix_msg):
        stamp = datetime.datetime.utcfromtimestamp(fix_msg.header.stamp.sec+fix_msg.header.stamp.nanosec*1e-9)
        #self.get_logger().info('utc_time: %s' % (str(stamp)))
        utc_time = str(stamp.hour) + str(stamp.minute) + str(stamp.second+stamp.microsecond*1e-6)
        #self.get_logger().info('utc_time: %s , %s ,%s' % (str(stamp.hour) , str(stamp.minute) , str(stamp.second+stamp.microsecond*1e-6)))
        lat_deg = str(abs(math.trunc(fix_msg.latitude)))
        lat_min = str(abs(fix_msg.latitude) % 1 * 60)
        lat_dir = ''
        if fix_msg.latitude > 0:
            lat_dir = 'N'
        else:
            lat_dir = 'S'



        lon_deg = str(abs(math.trunc(fix_msg.longitude)))
        lon_min = str(abs(fix_msg.longitude) % 1 * 60)
        lon_dir = ''
        if fix_msg.longitude > 0:
            lon_dir = 'E'
        else:
            lon_dir = 'W'
        quality = '1'
        num_sat = '24'
        hdop = '1.0'
        orthometric_height = str(fix_msg.altitude)
        geoid_separation = '0'
        age_of_gps = ''
        ref_station_id = ''

        msg = pynmea2.GGA('GP', 'GGA', (utc_time, lat_deg+lat_min, lat_dir, lon_deg+lon_min, lon_dir, quality, num_sat, hdop, orthometric_height, 'M', geoid_separation, 'M', age_of_gps, ref_station_id))

        self.gga_callback(str(msg)+str('\r\n'))

    def nmea_callback(self, nmea_msg):
        if nmea_msg.sentence.startswith(self.zda_id):
            self.zda_callback(nmea_msg.sentence)
        #if nmea_msg.sentence.startswith(self.gga_id):
        #    self.gga_callback(nmea_msg.sentence)

    def zda_callback(self, zda_sentence):
        # self.get_logger().info('encoding zda: %s' % zda_sentence)
        self.nmea_serial.write(zda_sentence.encode())

    def gga_callback(self, gga_sentence):
        # self.get_logger().info('encoding gga: %s' % gga_sentence)
        # self.get_logger().info('gga: %s' % (gga_sentence))
        self.nmea_serial.write(gga_sentence.encode())


def main(args=None):
    rclpy.init(args=args)

    kongsberg_interface = KongsbergInterface()

    rclpy.spin(kongsberg_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kongsberg_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
