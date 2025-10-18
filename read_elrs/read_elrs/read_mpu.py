#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import mpu6050
import math
import read_elrs.CONSTANTS as CONSTANTS
import numpy as np
from scipy.spatial.transform import Rotation


class ReadMPUNode(Node):
    def __init__(self):
        super().__init__("read_mpu")

        self.get_logger().info("Getting MPU data")

        self.mpu = mpu6050.mpu6050(0x68)

        self.publisher_IMU_data = self.create_publisher(Imu, "imu_data", 10)

        self.timer = self.create_timer(0.01, self.publish_imu_data)

        # Initialize orientation tracking
        self.last_time = self.get_clock().now()
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion [w,x,y,z]

    def calculate_orientation(self, accel_data, gyro_data, dt):
        """Calculate orientation quaternion using complementary filter"""

        # Convert accelerometer data to angles
        roll = math.atan2(accel_data['y'], accel_data['z'])
        pitch = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2))

        # Create rotation object from Euler angles
        accel_rotation = Rotation.from_euler('xyz', [roll, pitch, 0])
        accel_quat = accel_rotation.as_quat()  # [x,y,z,w]

        # Integrate gyroscope data
        gyro_rotation = Rotation.from_rotvec([
            gyro_data['x'] * dt,
            gyro_data['y'] * dt,
            gyro_data['z'] * dt
        ])
        
        current_rotation = Rotation.from_quat(self.current_orientation)
        gyro_updated = (current_rotation * gyro_rotation).as_quat()

        # Complementary filter
        alpha = 0.96  # Weight for gyroscope data
        self.current_orientation = alpha * gyro_updated + (1 - alpha) * accel_quat

        # Normalize quaternion
        self.current_orientation = self.current_orientation / np.linalg.norm(self.current_orientation)

        return self.current_orientation

    def publish_imu_data(self):
        try:
            # Get sensor data
            accel_data = self.mpu.get_accel_data()
            gyro_data = self.mpu.get_gyro_data()

            # Calculate time delta
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # Calculate orientation
            orientation = self.calculate_orientation(accel_data, gyro_data, dt)

            # Create Imu message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = CONSTANTS.imu_frame_id

            # Set orientation (quaternion)
            msg.orientation.x = float(orientation[0])
            msg.orientation.y = float(orientation[1])
            msg.orientation.z = float(orientation[2])
            msg.orientation.w = float(orientation[3])

            # Set linear acceleration
            msg.linear_acceleration.x = float(accel_data['x'])
            msg.linear_acceleration.y = float(accel_data['y'])
            msg.linear_acceleration.z = float(accel_data['z'])

            # Set angular velocity
            msg.angular_velocity.x = float(gyro_data['x'])
            msg.angular_velocity.y = float(gyro_data['y'])
            msg.angular_velocity.z = float(gyro_data['z'])

            # Set covariance matrices
            msg.orientation_covariance = [0.01, 0, 0,
                                        0, 0.01, 0,
                                        0, 0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0, 0,
                                             0, 0.01, 0,
                                             0, 0, 0.01]
            msg.linear_acceleration_covariance = [0.01, 0, 0,
                                                0, 0.01, 0,
                                                0, 0, 0.01]

            self.publisher_IMU_data.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading MPU6050: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ReadMPUNode()
    rclpy.spin(node)
    rclpy.shutdown()