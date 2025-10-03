#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16, Bool

import board
import neopixel
import random
import time

DATA_PIN = board.D10
pixels = neopixel.NeoPixel(DATA_PIN, 8, auto_write=False)

SBOld = 0
SCOld = 0
S1Old = 0
S2Old = 0

automatic = False
manual = False

R = False
G = False
B = False

RED = 0
GREEN = 0
BLUE = 0

class ControlLightsNode(Node):

    global pixels

    TOLERANCE = 10  # Adjust this value based on your needs

    def __init__(self):
        super().__init__("control_lights")
        self.get_logger().info("Node has been started")

        self.subscriberSB = self.create_subscription(Int16, "SB", self.callback_SB, 1)
        self.subscriberSC = self.create_subscription(Int16, "SC", self.callback_SC, 1)
        self.subscriberS1 = self.create_subscription(Int16, "S1", self.callback_S1, 1)
        self.subscriberS2 = self.create_subscription(Int16, "S2", self.callback_S2, 1)

        self.subscriberEmergencyFront = self.create_subscription(Bool, "emergency_front", self.callback_emergency_front, 1)
        self.subscriberEmergencyBack = self.create_subscription(Bool, "emergency_back", self.callback_emergency_back, 1)
        self.subscriberEmergencyLeft = self.create_subscription(Bool, "emergency_left", self.callback_emergency_left, 1)
        self.subscriberEmergencyRight = self.create_subscription(Bool, "emergency_right", self.callback_emergency_right, 1)

    # TODO check if new value is different from old value
    
    # 172 - 990 - 1800
    def callback_SB(self, msg: Int16):
        value = msg.data

        global SBOld
        global automatic
        global manual

        if value != SBOld:
            if 200 < value < 1000:
                automatic = True
                manual = False
                self.get_logger().info("Automatic mode lights")
            elif 1000 < value < 2000:
                automatic = False
                manual = True
                self.get_logger().info("Manual mode lights")

                self.all_pixels_color(RED, GREEN, BLUE)
            else:
                automatic = False
                manual = False
                self.get_logger().info("No lights")

                self.all_pixels_off()

        SBOld = value

    def callback_SC(self, msg: Int16):
        value = msg.data

        global SCOld

        global R, G, B

        if value != SCOld:
            if 200 < value < 1000:
                R = False
                G = True
                B = False
            elif 1000 < value < 2000:
                R = False
                G = False
                B = True
            else:
                R = True
                G = False
                B = False

        SCOld = value

    def callback_S1(self, msg: Int16):
        value = msg.data

    def callback_S2(self, msg: Int16):

        global S2Old, manual, RED, GREEN, BLUE, R, G, B

        value = msg.data

        if abs(value - S2Old) > self.TOLERANCE:

            if manual:
                if R:
                    RED = self.map_range(value)
                elif G:
                    GREEN = self.map_range(value)
                elif B:
                    BLUE = self.map_range(value)

                self.all_pixels_color(RED, GREEN, BLUE)

        S2Old = value

    def callback_emergency_front(self, msg: Bool):

        global automatic

        emergency_front = msg.data

        if automatic:
            if emergency_front:
                self.all_pixels_red()
            else:
                self.all_pixels_off()

    def callback_emergency_right(self, msg: Bool):

        global automatic

        emergency_right = msg.data

        if automatic:
            if emergency_right:
                self.all_pixels_red()
            else:
                self.all_pixels_off()

    def callback_emergency_back(self, msg: Bool):

        global automatic

        emergency_back = msg.data

        if automatic:
            if emergency_back:
                self.all_pixels_red()
            else:
                self.all_pixels_off()

    def callback_emergency_left(self, msg: Bool):

        global automatic

        emergency_left = msg.data

        if automatic:
            if emergency_left:
                self.all_pixels_red()
            else:
                self.all_pixels_off()


    def destroy_node(self):
        pixels.fill((0, 0, 0))
        pixels.show()

    def all_pixels_off(self):
        for i in range(8):
            pixels[i] = (0, 0, 0)
        pixels.show()

    def all_pixels_on(self):
        for i in range(8):
            pixels[i] = (255, 255, 255)
        pixels.show()

    def all_pixels_red(self):
        for i in range(8):
            pixels[i] = (255, 0, 0)
        pixels.show()

    def all_pixels_color(self, red, green, blue):
        for i in range(8):
            pixels[i] = (red, green, blue)
        pixels.show()

        # todo make rainbow circle move

    def circle_pixels(self, direction, duration=0.5):
        if direction == "clockwise":
            for i in range(8):
                pixels[i] = (255, 128, 0)
                pixels.show()
                time.sleep(duration / 8)
                pixels[i] = (0, 0, 0)
        elif direction == "counterclockwise":
            for i in range(7, -1, -1):
                pixels[i] = (255, 128, 0)
                pixels.show()
                time.sleep(duration / 8)
                pixels[i] = (0, 0, 0)

    def map_range(self, valoare, in_min=172, in_max=1800, out_min=0, out_max=255):

        valoare = max(in_min, min(valoare, in_max))

        mapped = (valoare - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

        return max(out_min, min(int(mapped), out_max))


def main(args=None):
    rclpy.init(args=args)
    node = ControlLightsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()