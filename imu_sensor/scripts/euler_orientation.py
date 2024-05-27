#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

class ImuToEulerConverter:
    def __init__(self):
        rospy.init_node('imu_to_euler_converter_node')
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.euler_pub = rospy.Publisher('/euler_topic', PoseStamped, queue_size=10)
        
        # Initialize variables
        self.initial_orientation_set = False
        self.initial_roll = 0.0
        self.initial_pitch = 0.0
        self.initial_yaw = 0.0

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)

        # If initial orientation is not set, set it
        if not self.initial_orientation_set:
            self.initial_roll = math.degrees(roll)
            self.initial_pitch = math.degrees(pitch)
            self.initial_yaw = math.degrees(yaw)
            self.initial_orientation_set = True

        # Calculate angular displacement from current orientation to initial orientation
        roll_diff = math.degrees(roll) - self.initial_roll
        pitch_diff = math.degrees(pitch) - self.initial_pitch
        yaw_diff = math.degrees(yaw) - self.initial_yaw

        # Normalize angles to be within -180 to 180 degrees
        roll_diff = self.normalize_angle(roll_diff)
        pitch_diff = self.normalize_angle(pitch_diff)
        yaw_diff = self.normalize_angle(yaw_diff)

        # Print Euler angles in degrees
        rospy.loginfo("Roll Diff: {:.2f} deg, Pitch Diff: {:.2f} deg, Yaw Diff: {:.2f} deg".format(roll_diff, pitch_diff, yaw_diff))

        # Create Euler angle message
        euler_msg = PoseStamped()
        euler_msg.header = msg.header
        euler_msg.pose.position.x = roll_diff
        euler_msg.pose.position.y = pitch_diff
        euler_msg.pose.position.z = yaw_diff

        # Publish Euler angle message
        self.euler_pub.publish(euler_msg)

if __name__ == '__main__':
    try:
        converter = ImuToEulerConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
