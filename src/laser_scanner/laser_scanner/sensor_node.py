import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import LaserScan

IR_PIN = 31  

class IRSensorNode(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.publisher_ = self.create_publisher(LaserScan, 'laser_scan', 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(IR_PIN, GPIO.IN)

        self.timer = self.create_timer(0.05, self.read_sensor)

    def read_sensor(self):
        obstacle = GPIO.input(IR_PIN) == 0 
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'main_laser'

        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 0.0
        msg.time_increment = 0.0
        msg.range_min = 0.0
        msg.range_max = 1.0

        msg.ranges = [1.0 - float(obstacle)]
        self.publisher_.publish(msg)

        if obstacle:
            self.get_logger().info(" IR sensor detected an obstacle!")
        else:
            self.get_logger().info(" No obstacle detected")


    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
