#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16
import lgpio

SA_old = 0
SD_old = 0
SE_old = 0
SF_old = 0

# Set the GPIO pin numbers
PIN_RELAY_1 = 6 # GPIO6
PIN_RELAY_2 = 13 # GPIO13
PIN_RELAY_3 = 19 # GPIO19
PIN_RELAY_4 = 26 # GPIO26

# Set the GPIO mode to BCM
h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PIN_RELAY_1, level=1)
lgpio.gpio_claim_output(h, PIN_RELAY_2, level=1)
lgpio.gpio_claim_output(h, PIN_RELAY_3, level=1)
lgpio.gpio_claim_output(h, PIN_RELAY_4, level=1)

class ControlRelaysNode(Node):

    global h
    global PIN_RELAY_1, PIN_RELAY_2, PIN_RELAY_3, PIN_RELAY_4

    def __init__(self):
        super().__init__("control_relays")
        self.get_logger().info("Control relays node started")

        self.subscriberSA = self.create_subscription(Int16, "SA", self.callback_SA, 1)
        self.subscriberSD = self.create_subscription(Int16, "SD", self.callback_SD, 1)
        self.subscriberSE = self.create_subscription(Int16, "SE", self.callback_SE, 1)
        self.subscriberSF = self.create_subscription(Int16, "SF", self.callback_SF, 1)

    def callback_SA(self, msg: Int16):

        global SA_old

        if msg.data != SA_old:

            if msg.data < 900:
                lgpio.gpio_write(h, PIN_RELAY_1, 0)

            elif msg.data > 1100:
                lgpio.gpio_write(h, PIN_RELAY_1, 1)

            SA_old = msg.data

    def callback_SD(self, msg: Int16):
        global SD_old

        if msg.data != SD_old:

            if msg.data < 900:
                lgpio.gpio_write(h, PIN_RELAY_2, 0)

            elif msg.data > 1100:
                lgpio.gpio_write(h, PIN_RELAY_2, 1)

            SD_old = msg.data

    def callback_SE(self, msg: Int16):
        global SE_old

        if msg.data != SE_old:

            if msg.data < 900:
                lgpio.gpio_write(h, PIN_RELAY_3, 0)

            elif msg.data > 1100:
                lgpio.gpio_write(h, PIN_RELAY_3, 1)

            SE_old = msg.data

    def callback_SF(self, msg: Int16):
        global SF_old

        if msg.data != SF_old:

            if msg.data < 900:
                lgpio.gpio_write(h, PIN_RELAY_4, 0)

            elif msg.data > 1100:
                lgpio.gpio_write(h, PIN_RELAY_4, 1)

            SF_old = msg.data


def main(args=None):

    rclpy.init(args=args)
    node = ControlRelaysNode()

    rclpy.spin(node)
    # Cleanup GPIO on shutdown
    lgpio.gpiochip_close(h)
    rclpy.shutdown()


if __name__ == "__main__":
    main()