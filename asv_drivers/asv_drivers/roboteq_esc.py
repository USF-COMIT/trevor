# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import Joy
import serial


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, bytesize=7, parity=serial.PARITY_EVEN, stopbits=1)  # open serial port

# The function defined bellow converts the integer values returned from the controller to hexidecimal
    def to_hex(self, value): # assume a value from -1,1
        scaled_value = abs(int(value*127))
        raw_hex = hex(scaled_value)[2:]
        if(len(raw_hex) == 1):
            raw_hex = "0" + raw_hex
        return raw_hex

    def listener_callback(self, msg):
        if msg.axes[1] < 0: #Controls the left thruster when in reverse
            cmd = "!a"+self.to_hex(msg.axes[1])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Left Thruster percent: %s cmd: %s' % (msg.axes[1]*100,cmd))
        if msg.axes[4] < 0: #Controls the left thruster when in reverse
            cmd = "!b"+self.to_hex(msg.axes[4])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Right Thruster percent: %s cmd: %s' % (msg.axes[4]*100,cmd))
        if msg.axes[4] > 0: #Controls the right thruster when driving forward
            cmd = "!B"+self.to_hex(msg.axes[4])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Right Thruster percent: %s cmd: %s' % (msg.axes[4]*100,cmd))
        if msg.axes[1] > 0: #Controls the left thruster when driving forward
            cmd = "!A"+self.to_hex(msg.axes[1])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Left Thruster percent: %s cmd: "%s"' % (msg.axes[1]*100,cmd))
        
        	
def main(args=None):
    rclpy.init(args=args)
    
    msg = Joy()

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
