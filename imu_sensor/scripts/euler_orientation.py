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
        self.initial_orientation = None

    def imu_callback(self, msg):
        if self.initial_orientation is None:
            # Store the initial orientation
            quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            self.initial_orientation = euler_from_quaternion(quat)

        # Convert quaternion to Euler angles
        quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Convert angles to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Calculate angular displacement relative to initial orientation
        roll_displacement = roll_deg - math.degrees(self.initial_orientation[0])
        pitch_displacement = pitch_deg - math.degrees(self.initial_orientation[1])
        yaw_displacement = yaw_deg - math.degrees(self.initial_orientation[2])

        # Print Euler angles in degrees
        rospy.loginfo("Roll: {:.2f} deg, Pitch: {:.2f} deg, Yaw: {:.2f} deg".format(roll_displacement, pitch_displacement, yaw_displacement))

        # Create Euler angle message
        euler_msg = PoseStamped()
        euler_msg.header = msg.header
        euler_msg.pose.position.x = roll_displacement
        euler_msg.pose.position.y = pitch_displacement
        euler_msg.pose.position.z = yaw_displacement

        # Publish Euler angle message
        self.euler_pub.publish(euler_msg)

if __name__ == '__main__':
    try:
        converter = ImuToEulerConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
