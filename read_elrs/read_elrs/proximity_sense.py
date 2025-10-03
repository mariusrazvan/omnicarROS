#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import lgpio
from example_interfaces.msg import Bool

ir_front_left = 20  # GPIO20 for the IR sensor
ir_front_right = 21  # GPIO21 for the IR sensor

ir_right_front = 16  # GPIO16 for the IR sensor
ir_right_back = 12  # GPIO12 for the IR sensor

ir_back_right = 5  # GPIO5 for the IR sensor
ir_back_left = 22  # GPIO22 for the IR sensor

ir_left_back = 24  # GPIO27 for the IR sensor
ir_left_front = 17  # GPIO17 for the IR sensor

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_input(h, ir_front_left)
lgpio.gpio_claim_input(h, ir_front_right)
lgpio.gpio_claim_input(h, ir_right_front)
lgpio.gpio_claim_input(h, ir_right_back)
lgpio.gpio_claim_input(h, ir_back_right)
lgpio.gpio_claim_input(h, ir_back_left)
lgpio.gpio_claim_input(h, ir_left_back)
lgpio.gpio_claim_input(h, ir_left_front)

class ProximitySenseNode(Node):


    def __init__(self):
        super().__init__("proximity_sense")

        self.get_logger().info("Sensing the proximity")

        self.publisher_emergency_front = self.create_publisher(Bool, "emergency_front", 1)
        self.publisher_emergency_back = self.create_publisher(Bool, "emergency_back", 1)
        self.publisher_emergency_left = self.create_publisher(Bool, "emergency_left", 1)
        self.publisher_emergency_right = self.create_publisher(Bool, "emergency_right", 1)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def publish_emergency_front(self, urgenta1):
        msg = Bool()
        msg.data = urgenta1
        self.publisher_emergency_front.publish(msg)

    def publish_emergency_back(self, urgenta2):
        msg = Bool()
        msg.data = urgenta2
        self.publisher_emergency_back.publish(msg)

    def publish_emergency_left(self, urgenta3):
        msg = Bool()
        msg.data = urgenta3
        self.publisher_emergency_left.publish(msg)

    def publish_emergency_right(self, urgenta4):
        msg = Bool()
        msg.data = urgenta4
        self.publisher_emergency_right.publish(msg)

    def timer_callback(self):

        global h

        global ir_front_right
        global ir_front_left

        global ir_right_front
        global ir_right_back

        global ir_back_right
        global ir_back_left

        global ir_left_back
        global ir_left_front

        if lgpio.gpio_read(h, ir_front_left) == 0 or lgpio.gpio_read(h, ir_front_right) == 0:
            urgenta_front = True
        else:
            urgenta_front = False

        if lgpio.gpio_read(h, ir_right_front) == 0 or lgpio.gpio_read(h, ir_right_back) == 0:
            urgebta_right = True
        else:
            urgebta_right = False

        if lgpio.gpio_read(h, ir_back_right) == 0 or lgpio.gpio_read(h, ir_back_left) == 0:
            urgenta_back = True
        else:
            urgenta_back = False

        if lgpio.gpio_read(h, ir_left_back) == 0 or lgpio.gpio_read(h, ir_left_front) == 0:
            urgenta_left = True
        else:
            urgenta_left = False

        self.publish_emergency_front(urgenta_front)
        self.publish_emergency_right(urgebta_right)
        self.publish_emergency_back(urgenta_back)
        self.publish_emergency_left(urgenta_left)
        

def main(args=None):
    rclpy.init(args=args)
    node = ProximitySenseNode()
    rclpy.spin(node)
    rclpy.shutdown()
    lgpio.gpiochip_close(h)