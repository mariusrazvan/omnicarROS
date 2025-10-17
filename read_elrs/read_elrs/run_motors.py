#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from read_elrs.Emakefun_MotorHAT import Emakefun_MotorHAT
from example_interfaces.msg import Bool
from math import atan2, sqrt, sin, cos, pi as PI
import read_elrs.CONSTANTS as CONSTANTS

cmd_vel = Twist()

motorHat = Emakefun_MotorHAT()

frontLeftMotor = motorHat.getMotor(1)
frontRightMotor = motorHat.getMotor(2)
rearRightMotor = motorHat.getMotor(3)
rearLeftMotor = motorHat.getMotor(4)

emergency_front = False
emergency_back = False
emergency_left = False
emergency_right = False

class RunMotorsNode(Node):

    global motoru

    def __init__(self):
        super().__init__("run_motors")
        self.get_logger().info("Run motors node started")

        self.subscriber_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.callback_Cmd_Vel, 1)

        self.subscriberEmergencyFront = self.create_subscription(Bool, "emergency_front", self.callback_emergency_front, 1)
        self.subscriberEmergencyBack = self.create_subscription(Bool, "emergency_back", self.callback_emergency_back, 1)
        self.subscriberEmergencyLeft = self.create_subscription(Bool, "emergency_left", self.callback_emergency_left, 1)
        self.subscriberEmergencyRight = self.create_subscription(Bool, "emergency_right", self.callback_emergency_right, 1)


    def callback_Cmd_Vel(self, msg: Twist):

        global cmd_vel

        cmd_vel = msg

        self.combined_movement(cmd_vel)

    def callback_emergency_front(self, msg: Bool):

        global emergency_front

        emergency_front = msg.data

    def callback_emergency_right(self, msg: Bool):

        global emergency_right

        emergency_right = msg.data

    def callback_emergency_back(self, msg: Bool):

        global emergency_back

        emergency_back = msg.data

    def callback_emergency_left(self, msg: Bool):

        global emergency_left

        emergency_left = msg.data

    def combined_movement(self, cmd_vel):

        global frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor
        global emergency_front, emergency_back, emergency_left, emergency_right

        #TODO handle emergency stops with a break first!

        # Extract cmd_vel values

        x = self.map_range_linear(cmd_vel.linear.x)   # Forward/Backward from m/s
        y = self.map_range_linear(cmd_vel.linear.y)   # Left/Right from m/s

        r = self.map_range_angular(cmd_vel.angular.z)  # Rotation from rad/s

        angle = atan2(y, x) if (x != 0 or y != 0) else 0
        power = sqrt(x**2 + y**2)
        rotation = r

        # Calculate wheel speeds
        wheelFL = power * sin(angle + (PI / 4)) + rotation
        wheelFR = power * cos(angle + (PI / 4)) - rotation
        wheelBL = power * cos(angle + (PI / 4)) + rotation
        wheelBR = power * sin(angle + (PI / 4)) - rotation

        if wheelFL > 0:
            frontLeftMotor.run(Emakefun_MotorHAT.FORWARD)
        else:
            frontLeftMotor.run(Emakefun_MotorHAT.BACKWARD)

        frontLeftMotor.setSpeed(abs(int(wheelFL)))

        if wheelFR > 0:
            frontRightMotor.run(Emakefun_MotorHAT.FORWARD)
        else:
            frontRightMotor.run(Emakefun_MotorHAT.BACKWARD)

        frontRightMotor.setSpeed(abs(int(wheelFR)))

        if wheelBL > 0:
            rearLeftMotor.run(Emakefun_MotorHAT.FORWARD)
        else:
            rearLeftMotor.run(Emakefun_MotorHAT.BACKWARD)

        rearLeftMotor.setSpeed(abs(int(wheelBL)))

        if wheelBR > 0:
            rearRightMotor.run(Emakefun_MotorHAT.FORWARD)
        else:
            rearRightMotor.run(Emakefun_MotorHAT.BACKWARD)

        rearRightMotor.setSpeed(abs(int(wheelBR)))

    def stop_motors(self):
        global frontLeftMotor
        global frontRightMotor
        global rearLeftMotor
        global rearRightMotor

        frontLeftMotor.run(Emakefun_MotorHAT.BRAKE)
        frontRightMotor.run(Emakefun_MotorHAT.BRAKE)
        rearLeftMotor.run(Emakefun_MotorHAT.BRAKE)
        rearRightMotor.run(Emakefun_MotorHAT.BRAKE)

    def relantiu(self):
        global frontLeftMotor
        global frontRightMotor
        global rearLeftMotor
        global rearRightMotor

        frontLeftMotor.run(Emakefun_MotorHAT.RELEASE)
        frontRightMotor.run(Emakefun_MotorHAT.RELEASE)
        rearLeftMotor.run(Emakefun_MotorHAT.RELEASE)
        rearRightMotor.run(Emakefun_MotorHAT.RELEASE)

    # change from m/s to motor speed range -255 to 255
    def map_range_linear(self, value, old_min=-CONSTANTS.max_linear_velocity, old_max=CONSTANTS.max_linear_velocity, new_min=-255, new_max=255):

        if old_min <= value and value <= old_max:

            scaled = (value - old_min) / (old_max - old_min)
            mapped_value = scaled * (new_max - new_min) + new_min

        else:
            mapped_value = 0.0

        return mapped_value
    
    def map_range_angular(self, value, old_min=-CONSTANTS.max_angular_velocity, old_max=CONSTANTS.max_angular_velocity, new_min=-255, new_max=255):

        if old_min <= value and value <= old_max:

            scaled = (value - old_min) / (old_max - old_min)
            mapped_value = scaled * (new_max - new_min) + new_min

        else:
            mapped_value = 0.0

        return mapped_value


def main(args=None):

    rclpy.init(args=args)
    node = RunMotorsNode()
    rclpy.spin(node)
    rclpy.shutdown()