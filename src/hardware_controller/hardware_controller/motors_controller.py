import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class CommandNode(Node):
    def __init__(self):
        super().__init__('motors_controller')


        # Initialize serial communication with Arduino
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial connection established with Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None
        
        # Create a subscriber to the 'distance' topic
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        

    def listener_callback(self, msg: Twist) -> None:
        if msg.linear.x < 0:
            command = 'BACKWARD\n'
            self.get_logger().info("Obstacle detected, moving backward.")
        elif msg.linear.x == 0:
            if msg.angular.z == 0:
                command = 'STOP\n'
                self.get_logger().info("No signal from sensor, stopping.")
        else:
            command = 'FORWARD\n'
            self.get_logger().info("No obstacle detected, moving forward.")
        
        if self.ser:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent command: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
