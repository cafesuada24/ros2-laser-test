import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool

class CommandNode(Node):
    def __init__(self):
        super().__init__('motor_controller')


        # Initialize serial communication with Arduino
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial connection established with Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None
        
        # Create a subscriber to the 'distance' topic
        self.subscription = self.create_subscription(Bool, 'obstacle_detected', self.listener_callback, 10)
        

    def listener_callback(self, msg):
        if msg.data:
            command = 'STOP\n'
            self.get_logger().info(f"Obstacle detected, stopping motors.")
        else:
            command = 'FORWARD\n'
            self.get_logger().info(f"No obstacle detected, moving forward.")
        
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
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()
