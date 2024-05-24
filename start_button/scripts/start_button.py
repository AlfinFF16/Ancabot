#!/usr/bin/env python3

import rospy
import subprocess
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Define GPIO pin for the push button
BUTTON_PIN = 11

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)                                      # BOARD pin-numbering scheme
GPIO.setup(BUTTON_PIN, GPIO.IN)                               # button pin set as input


def button_callback(channel):
    rospy.loginfo("Button pressed!")
    # Launch your ROS launch file here
    subprocess.call("/home/jetson/ancabot_ws/src/shell_script/navigation.sh", shell=True)
    # Publish a message to the 'button_pressed' topic
    pub.publish(Bool(True))

# Initialize the ROS node
rospy.init_node('button_controller_node')

# Set up the publisher for the button press
pub = rospy.Publisher('button_pressed', Bool, queue_size=10)

# Subscribe to button topic
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

rospy.loginfo("Button controller node ready...")

rospy.spin()

# Clean up GPIO
GPIO.cleanup()

