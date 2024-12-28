#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import Adafruit_PCA9685  # Library to control the PCA9685
import tf.transformations as tf

# Example: Importing a library for the IMU sensor (Replace this with your actual IMU library)
from Adafruit_BNO055 import BNO055  # Replace with your specific IMU library

class RobotDriverNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_driver_node', anonymous=True)

        # Initialize PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50 Hz for servos

        # Initialize IMU sensor
        self.imu = BNO055.BNO055(serial_port='/dev/ttyUSB0')  # Update serial_port based on your setup
        if not self.imu.begin():
            rospy.logerr("Failed to initialize the IMU sensor!")
            raise RuntimeError("IMU initialization failed")

        rospy.loginfo("IMU sensor initialized successfully.")

        # Joint angles (12 servos assumed)
        self.joint_angles = [0] * 12  # Initial positions

        # Subscribe to joint command topics
        self.command_topics = [
            "/notspot_controller/FR1_joint/command",
            "/notspot_controller/FR2_joint/command",
            "/notspot_controller/FR3_joint/command",
            "/notspot_controller/FL1_joint/command",
            "/notspot_controller/FL2_joint/command",
            "/notspot_controller/FL3_joint/command",
            "/notspot_controller/RR1_joint/command",
            "/notspot_controller/RR2_joint/command",
            "/notspot_controller/RR3_joint/command",
            "/notspot_controller/RL1_joint/command",
            "/notspot_controller/RL2_joint/command",
            "/notspot_controller/RL3_joint/command",
        ]

        for i, topic in enumerate(self.command_topics):
            rospy.Subscriber(topic, Float64, self.joint_command_callback, callback_args=i)

        # Publish IMU data
        self.imu_publisher = rospy.Publisher("notspot_imu/base_link_orientation", Imu, queue_size=10)

        # Timer to read and publish IMU data
        rospy.Timer(rospy.Duration(0.04), self.publish_imu_data)  # 25 Hz

    def joint_command_callback(self, msg, joint_index):
        """Callback for receiving joint angles."""
        self.joint_angles[joint_index] = msg.data
        self.update_servo(joint_index, msg.data)

    def update_servo(self, joint_index, angle):
        """Update the servo position based on joint angle."""
        # Convert angle (e.g., -90 to 90 degrees) to PWM pulse (e.g., 150 to 600 for PCA9685)
        min_pulse = 150
        max_pulse = 600
        pulse = int(min_pulse + (angle + 90) / 180.0 * (max_pulse - min_pulse))
        self.pwm.set_pwm(joint_index, 0, pulse)

    def publish_imu_data(self, event):
        """Publish IMU data from the sensor."""
        try:
            # Get IMU orientation data
            quaternion = self.imu.read_quaternion()  # Replace with appropriate method for your IMU
            angular_velocity = self.imu.read_gyroscope()  # Replace with appropriate method for gyroscope
            linear_acceleration = self.imu.read_accelerometer()  # Replace with appropriate method for accelerometer

            # Prepare IMU message
            msg = Imu()
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]
            msg.angular_velocity.x = angular_velocity[0]
            msg.angular_velocity.y = angular_velocity[1]
            msg.angular_velocity.z = angular_velocity[2]
            msg.linear_acceleration.x = linear_acceleration[0]
            msg.linear_acceleration.y = linear_acceleration[1]
            msg.linear_acceleration.z = linear_acceleration[2]

            # Transform quaternion to RPY for internal state if needed
            rpy_angles = tf.euler_from_quaternion(quaternion)
            rospy.loginfo(f"IMU Roll: {rpy_angles[0]}, Pitch: {rpy_angles[1]}, Yaw: {rpy_angles[2]}")

            # Publish the IMU message
            self.imu_publisher.publish(msg)

        except Exception as e:
            rospy.logerr(f"Failed to read IMU data: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        driver_node = RobotDriverNode()
        driver_node.run()
    except rospy.ROSInterruptException:
        pass
