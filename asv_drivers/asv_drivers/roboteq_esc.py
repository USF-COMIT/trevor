
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import Joy
import serial


class RoboteqEsc(Node):

    def __init__(self):
        super().__init__('roboteq_esc')

        self.declare_parameter('joy_topic'  , 'joy')
        self.declare_parameter('esc_port'   , '/dev/ttyUSB0')

        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.esc_port  = self.get_parameter('esc_port').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(port=self.esc_port, baudrate=9600, bytesize=7, parity=serial.PARITY_EVEN, stopbits=1)  # open serial port

        self.get_logger().info('Sending ESC commands on port: %s' % (self.esc_port))


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
        if msg.axes[4] >= 0: #Controls the right thruster when driving forward
            cmd = "!B"+self.to_hex(msg.axes[4])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Right Thruster percent: %s cmd: %s' % (msg.axes[4]*100,cmd))
        if msg.axes[1] >= 0: #Controls the left thruster when driving forward
            cmd = "!A"+self.to_hex(msg.axes[1])+"\r"
            self.ser.write(cmd.encode())
            self.get_logger().info('Left Thruster percent: %s cmd: "%s"' % (msg.axes[1]*100,cmd))
        
        	
def main(args=None):
    rclpy.init(args=args)
    
    msg = Joy()

    esc = RoboteqEsc()

    rclpy.spin(esc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
