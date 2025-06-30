from math import inf
from typing import List, Optional
import Jetson.GPIO as GPIO
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
# import time

GPIO.setmode(GPIO.BOARD)


class UltrasonicSensorNode(Node):
    MAX_OBSTACLE_DISTANCE: float = 10

    def __init__(
        self, node_name: str = "ultrasonic", pin_trig: int = 31, pin_echo: int = 33
    ) -> None:
        super().__init__(node_name=node_name)
        self.__id = f"{node_name}/raw_scan"

        self.__pin_trig = pin_trig
        self.__pin_echo = pin_echo

        try:
            GPIO.setup(self.__pin_trig, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.__pin_echo, GPIO.IN)
        except:
            rclpy.shutdown()
            raise

        # self.declare_parameters(
        #     namespace="GPIOPin",
        #     parameters=[
        #         ("trig", self.__pin_trig, "Trigger pin"),
        #         ("echo", self.__pin_echo, "Echo pin"),
        #     ],
        # )
        GPIO.setup(self.__pin_trig, GPIO.OUT, initial=0)
        # self.add_on_set_parameters_callback(self.__set_parameters_callback)

        self.__scan_publisher = self.create_publisher(LaserScan, self.__id, 10)
        self.create_timer(0.1, self.__publish_distance)
        self.__sleep_time = Duration(seconds=0.00002)

    def __publish_distance(self) -> None:
        msg = LaserScan()
        msg.header = Header()
        msg.header.frame_id = self.__id

        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 0.0
        msg.time_increment = 0.0
        msg.range_min = 0.0
        msg.range_max = 10.0  # Meters

        try:
            dist = self.__get_obstacle_distance()
            msg.ranges = [dist]
            msg.header.stamp = self.get_clock().now().to_msg()
            if dist != inf:
                self.get_logger().info(f"Obstacle dected at [{dist} m].")
            else:
                self.get_logger().info("No obstacle detected")
        except Exception as e:
            self.get_logger().error(str(e))
            return

        self.__scan_publisher.publish(msg)

    def __get_obstacle_distance(self) -> float:
        """Get measured distance in meter."""
        GPIO.output(self.__pin_trig, GPIO.HIGH)
        target_time = self.get_clock().now() + self.__sleep_time
        while self.get_clock().now() < target_time and rclpy.ok():
            pass
        # self.get_clock().sleep_until(self.get_clock().now() + 0.00001)
        GPIO.output(self.__pin_trig, GPIO.LOW)

        # startTime = self.get_clock().now()
        # arrivalTime = self.get_clock().now()

        # start_time = time.time()
        start_time = self.get_clock().now().seconds_nanoseconds()[1]
        while GPIO.input(self.__pin_echo) == 0:
            # start_time = time.time()
            start_time = self.get_clock().now().seconds_nanoseconds()[1]
        # arrival_time = time.time()
        arrival_time = self.get_clock().now().seconds_nanoseconds()[1]
        while GPIO.input(self.__pin_echo) == 1:
            # arrival_time = time.time()
            arrival_time = self.get_clock().now().seconds_nanoseconds()[1]

        time_elapsed = (arrival_time - start_time) / 1000000000
        distance = round(time_elapsed * 17150 / 100, 4)
        distance = distance if distance <= self.MAX_OBSTACLE_DISTANCE else inf
        return distance

    # def __set_parameters_callback(
    #     self, parameters: list[Parameter]
    # ) -> SetParametersResult:
    #     result = SetParametersResult()
    #     result.successful = True
    #     for param in parameters:
    #         try:
    #             match param.name:
    #                 case "trig":
    #                     value = param.get_parameter_value().integer_value()
    #                     if not isinstance(value, int):
    #                         raise ValueError("Invalid TRIG GPIO pin")
    #                     GPIO.setup(value, GPIO.OUT, initial=0)
    #                 case "echo":
    #                     value = param.get_parameter_value().integer_value()
    #                     if not isinstance(value, int):
    #                         raise ValueError("Invalid ECHO GPIO pin")
    #                     GPIO.setup(value, GPIO.IN)
    #         except Exception as e:
    #             result.successful = False
    #             result.reason = str(e)
    #             return result
    #     return result


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()
