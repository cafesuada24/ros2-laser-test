import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class CommandNode(Node):
    def __init__(self):
        super().__init__("motors_controller")

        # Initialize serial communication with Arduino
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
            self.get_logger().info("Serial connection established with Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None
            raise

        self.last_ctl_command: Twist | None = None
        self.timer = self.create_timer(0.05, self.motor_control)

        # Create a subscriber to the 'distance' topic
        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.listener_callback,
            qos_profile_sensor_data,
        )

    def motor_control(self) -> None:
        if self.last_ctl_command is None:
            return

        cmd = self.last_ctl_command
        self.last_ctl_command = None

        if cmd.linear is not None and cmd.linear.x != 0:
            if cmd.linear.x < 0:
                command = "BACKWARD\n"
                self.get_logger().info("Obstacle detected, moving backward.")
            # elif cmd.linear.x == 0:
            #     if cmd.angular.z == 0:
            #         command = 'STOP\n'
            #         self.get_logger().info("No signal from sensor, stopping.")
            else:
                command = "FORWARD\n"
                self.get_logger().info("No obstacle detected, moving forward.")
        elif cmd.angular is not None and cmd.angular.z != 0:
            if cmd.angular.z > 0:
                command = "LEFT\n"
                self.get_logger().info("Turning left")
            # elif cmd.linear.x == 0:
            #     if cmd.angular.z == 0:
            #         command = 'STOP\n'
            #         self.get_logger().info("No signal from sensor, stopping.")
            else:
                command = "RIGHT\n"
                self.get_logger().info("Turning right")
        else:
            command = "STOP\n"
            self.get_logger().info("Stopping...")

        if self.ser:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent command: {command.strip()}")

    def listener_callback(self, msg: Twist) -> None:
        self.last_ctl_command = msg


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
