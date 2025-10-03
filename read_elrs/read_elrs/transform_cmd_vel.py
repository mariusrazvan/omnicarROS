#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16
from geometry_msgs.msg import Twist
import math
import CONSTANTS


LJVertical = 0
LJHorizontal = 0
RJHorizontal = 0


class TransformCmdVelNode(Node):
    def __init__(self):
        super().__init__("transform_cmd_vel")
        self.get_logger().info("Transforming from remote to cmd_vel")

        self.subscriberLJVertical = self.create_subscription(Int16, "LJVertical", self.callback_LJVertical, 1)
        self.subscriberLJHorizontal = self.create_subscription(Int16, "LJHorizontal", self.callback_LJHorizontal, 1)
        self.subscriberRJHorizontal = self.create_subscription(Int16, "RJHorizontal", self.callback_RJHorizontal, 1)

        self.publisher_cmd_vel = self.create_publisher(Twist, "cmd_vel", 1)

    def callback_LJVertical(self, msg: Int16):
        
        global LJVertical

        # between 172 and 1810
        # 990 is the center position
        LJVertical = msg.data

        self.transformer()

    def callback_LJHorizontal(self, msg: Int16):
        
        global LJHorizontal

        # between 172 and 1810
        # 990 is the center position
        LJHorizontal = msg.data

        self.transformer()

    def callback_RJHorizontal(self, msg: Int16):
        
        global RJHorizontal

        # between 172 and 1810
        # 990 is the center position
        RJHorizontal = msg.data

        self.transformer()

    def transformer(self):

        global LJVertical
        global LJHorizontal
        global RJHorizontal

        global wheel_radius
        global motor_rpm_max

        cmd_vel = Twist()

        remote_x_to_rpm = self.map_range(LJVertical)    # maps to the motor rpm range
        remote_y_to_rpm = self.map_range(LJHorizontal)
        remote_z_to_rpm = self.map_range(RJHorizontal)

        if abs(remote_x_to_rpm) < 5.0:
            remote_x_to_rpm = 0.0

        if abs(remote_y_to_rpm) < 5.0:
            remote_y_to_rpm = 0.0

        if abs(remote_z_to_rpm) < 5.0:
            remote_z_to_rpm = 0.0

        linear_x = self.get_meters_per_second(remote_x_to_rpm, CONSTANTS.wheel_radius) # in m/s
        linear_y = self.get_meters_per_second(remote_y_to_rpm, CONSTANTS.wheel_radius) # in m/s

        angular_z = self.get_angular_velocity(remote_z_to_rpm) # in rad/s

        cmd_vel.linear.x = linear_x # forward/backward
        cmd_vel.linear.y = linear_y # strife left/right
        cmd_vel.linear.z = 0.0    # up/down

        cmd_vel.angular.z = angular_z # rotate left/right
        cmd_vel.angular.x = 0.0   # tilt forward/backward
        cmd_vel.angular.y = 0.0   # tilt left/right

        self.publisher_cmd_vel.publish(cmd_vel)

    def get_meters_per_second(self, rpm: float, wheel_radius_m: float):
        
        rotations_per_second = rpm / 60.0
        circumference = 2 * math.pi * wheel_radius_m
        speed_mps = rotations_per_second * circumference

        return speed_mps
    
    def get_angular_velocity(self, rpm: float):

        angular_velocity = (rpm * 2 * math.pi) / 60.0

        return angular_velocity

    def map_range(self, value, old_min=172, old_max=1810, new_min=-CONSTANTS.motor_rpm_max, new_max=CONSTANTS.motor_rpm_max):

        if old_min <= value and value <= old_max:

            scaled = (value - old_min) / (old_max - old_min)
            mapped_value = scaled * (new_max - new_min) + new_min

        else:
            mapped_value = 0.0

        return mapped_value
        

def main(args=None):
    rclpy.init(args=args)
    node = TransformCmdVelNode()
    rclpy.spin(node)
    rclpy.shutdown()