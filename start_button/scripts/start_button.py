#!/usr/bin/env python3

import rospy
import subprocess
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
    subprocess.call("/home/jetson/ancabot_ws/src/shell_script/navigation.sh", shell=True)
    # Publish a message to the 'button_pressed' topic
    pub.publish(Bool(True))

def main():
    # Initialize the ROS node
    rospy.init_node('button_controller_node')

    # Set up the publisher for the button press
    pub = rospy.Publisher('button_pressed', Bool, queue_size=10)

    # Subscribe to button topic with debounce time increased to 300 ms
    GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

    rospy.loginfo("Button controller node ready...")

    # Keep the script running and checking for events
    try:
        rospy.spin()
    finally:
        GPIO.cleanup()  # Clean up GPIO on exit

if __name__ == '__main__':
    main()

