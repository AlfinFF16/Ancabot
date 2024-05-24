#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Define GPIO pin for the push button
BUTTON_PIN = 11

# Initialize GPIO
GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin set as input with pull-up resistor

def button_callback(channel):
    rospy.loginfo("Button pressed!")
    # Publish a message to the 'button_pressed' topic
    pub.publish(Bool(True))

# Initialize the ROS node
rospy.init_node('button_controller_node')

# Set up the publisher for the button press
pub = rospy.Publisher('button_pressed', Bool, queue_size=10)

# Subscribe to button topic with debounce time increased to 300 ms
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

rospy.loginfo("Button controller node ready...")

# Polling loop to keep the script running and checking for events
try:
    while not rospy.is_shutdown():
        rospy.sleep(0.1)  # Sleep for 100 ms to reduce CPU usage
finally:
    GPIO.cleanup()  # Clean up GPIO on exit

rospy.spin()

