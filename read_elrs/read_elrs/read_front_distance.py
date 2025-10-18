#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg.Range import Range
import VL53L0X
import math
import read_elrs.CONSTANTS as CONSTANTS

tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

class ReadFrontDistanceNode(Node):
    def __init__(self):
        super().__init__("read_front_distance")
        self.get_logger().info("ReadFrontDistanceNode has been started")

        self.publisher_front_distance = self.create_publisher(Range, "scan", 1)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):

        distance = tof.get_distance() / 1000.0  # Convert mm to meters

        if distance > 0:
            self.publish_front_distance(distance)

    def publish_front_distance(self, distance):
        msg = Range()

        msg.header.frame_id = CONSTANTS.tof_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = math.radians(25.0)
        
        msg.min_range = 0.005
        msg.max_range = 2.0

        msg.range = distance

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