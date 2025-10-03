#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16
import VL53L0X

tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof.get_timing()
if timing < 20000:
    timing = 20000

class ReadFrontDistanceNode(Node):
    def __init__(self):
        super().__init__("read_front_distance")
        self.get_logger().info("ReadFrontDistanceNode has been started")

        self.publisher_front_distance = self.create_publisher(Int16, "scan", 1)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):

        distance = tof.get_distance()

        if distance > 0:
            self.publish_front_distance(distance)

    def publish_front_distance(self, distance):
        msg = Int16()
        msg.data = distance
        self.publisher_front_distance.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReadFrontDistanceNode()
    rclpy.spin(node)
    tof.stop_ranging()
    tof.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()