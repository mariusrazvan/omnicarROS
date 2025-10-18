#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16
import lgpio

SF_old = 0

BEEPING_PIN = 27  # GPIO27
FREQ = 1000

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, BEEPING_PIN, 0)


class ControlBeepNode(Node):
    def __init__(self):
        super().__init__("control_beep")
        self.get_logger().info("ControlBeepNode has been started")
        self.subscriberSF = self.create_subscription(Int16, "SF", self.callback_SF, 1)

    def callback_SF(self, msg: Int16):
        global SF_old

        if msg.data != SF_old:
            if msg.data > 1100:
                lgpio.tx_pwm(h, BEEPING_PIN, FREQ, 0.0)
                self.get_logger().info("Beep stopped")

            elif msg.data < 900:
                lgpio.tx_pwm(h, BEEPING_PIN, FREQ, 100.0)
                self.get_logger().info("Beep started")

            SF_old = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = ControlBeepNode()
    rclpy.spin(node)
    lgpio.gpiochip_close(h)
    rclpy.shutdown()

if __name__ == "__main__":
    main()