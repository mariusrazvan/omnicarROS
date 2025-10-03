#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16

from typing import Container
from crsf_parser import CRSFParser, PacketValidationStatus
from serial import Serial

node = None

channelsValues = []

SA = 0
SB = 0
SC = 0
SD = 0
SE = 0
SF = 0

S1 = 0
S2 = 0

LJVertical = 0
LJHorizontal = 0
RJVertical = 0
RJHorizontal = 0


class ReadRemoteNode(Node):
    def __init__(self):
        super().__init__("read_remote")
        self.get_logger().info("Reading the remote...")

        self.publisher_SA = self.create_publisher(Int16, "SA", 1)
        self.publisher_SB = self.create_publisher(Int16, "SB", 1)
        self.publisher_SC = self.create_publisher(Int16, "SC", 1)
        self.publisher_SD = self.create_publisher(Int16, "SD", 1)
        self.publisher_SE = self.create_publisher(Int16, "SE", 1)
        self.publisher_SF = self.create_publisher(Int16, "SF", 1)

        self.publisher_S1 = self.create_publisher(Int16, "S1", 1)
        self.publisher_S2 = self.create_publisher(Int16, "S2", 1)

        self.publisher_LJVertical = self.create_publisher(Int16, "LJVertical", 1)
        self.publisher_LJHorizontal = self.create_publisher(Int16, "LJHorizontal", 1)
        self.publisher_RJVertical = self.create_publisher(Int16, "RJVertical", 1)
        self.publisher_RJHorizontal = self.create_publisher(Int16, "RJHorizontal", 1)

        self.timer = self.create_timer(0.01, self.timer_callback)  # in secunde

    def timer_callback(self):
        readRemote()

    def publish_SA(self):
        msg = Int16()
        msg.data = SA
        self.publisher_SA.publish(msg)

    def publish_SB(self):
        msg = Int16()
        msg.data = SB
        self.publisher_SB.publish(msg)

    def publish_SC(self):
        msg = Int16()
        msg.data = SC
        self.publisher_SC.publish(msg)

    def publish_SD(self):
        msg = Int16()
        msg.data = SD
        self.publisher_SD.publish(msg)

    def publish_SE(self):
        msg = Int16()
        msg.data = SE
        self.publisher_SE.publish(msg)

    def publish_SF(self):
        msg = Int16()
        msg.data = SF
        self.publisher_SF.publish(msg)

    def publish_S1(self):
        msg = Int16()
        msg.data = S1
        self.publisher_S1.publish(msg)

    def publish_S2(self):
        msg = Int16()
        msg.data = S2
        self.publisher_S2.publish(msg)

    def publish_LJVertical(self):
        msg = Int16()
        msg.data = LJVertical
        self.publisher_LJVertical.publish(msg)
    
    def publish_LJHorizontal(self):
        msg = Int16()
        msg.data = LJHorizontal
        self.publisher_LJHorizontal.publish(msg)

    def publish_RJVertical(self):
        msg = Int16()
        msg.data = RJVertical
        self.publisher_RJVertical.publish(msg)

    def publish_RJHorizontal(self):
        msg = Int16()
        msg.data = RJHorizontal
        self.publisher_RJHorizontal.publish(msg)


def main(args=None):

    global node

    rclpy.init(args=args)
    node = ReadRemoteNode()
    rclpy.spin(node)
    rclpy.shutdown()

def get_frame_values(frame: Container, status: PacketValidationStatus):

    global node

    global channelsValues

    global SA
    global SB
    global SC
    global SD
    global SE
    global SF

    global S1
    global S2

    global LJVertical
    global LJHorizontal
    global RJVertical
    global RJHorizontal

    if status and frame.get("payload").get("channels") != None:

        channelsValues.clear()
        channelsValues.extend(frame.get("payload").get("channels"))

        #print(frame.get("payload").get("channels"))

        SA = channelsValues[11]
        SB = channelsValues[10]
        SC = channelsValues[9]
        SD = channelsValues[8]
        SE = channelsValues[7]
        SF = channelsValues[6]

        S1 = channelsValues[5]
        S2 = channelsValues[4]
        
        LJVertical = channelsValues[14]
        LJHorizontal = channelsValues[12]
        RJVertical = channelsValues[13]
        RJHorizontal = channelsValues[15]

        node.publish_SA()
        node.publish_SB()
        node.publish_SC()
        node.publish_SD()
        node.publish_SE()
        node.publish_SF()

        node.publish_S1()
        node.publish_S2()
        
        node.publish_LJVertical()
        node.publish_LJHorizontal()
        node.publish_RJVertical()
        node.publish_RJHorizontal()




def readRemote():

    crsf_parser = CRSFParser(get_frame_values)

    with Serial("/dev/ttyAMA0", 420000, timeout=1) as ser:

        input = bytearray()

        while True:
        
            values = ser.read(100)

            input.extend(values)

            crsf_parser.parse_stream(input)