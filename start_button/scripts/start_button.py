#!/usr/bin/env python3

import rospy
import os
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Define GPIO pin for the push button
BUTTON_PIN = 11

package = "hexapod_bringup"
executable = "ancabot_start.launch"

def button_callback(channel):
    try:
        rospy.loginfo("Button pressed!")
        # Launch your ROS launch file here
        os.system(f"roslaunch {package} {executable}")
    except Exception as e:
        rospy.logerr("Error occurred while launching: %s", str(e))

def main():
    rospy.init_node('button_controller_node', anonymous=True)

    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(BUTTON_PIN, GPIO.IN)  # button pin set as input
    
    # Subscribe to button topic
    GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

    rospy.loginfo("Button controller node ready...")

    rospy.spin()

    # Clean up GPIO
    GPIO.cleanup()

if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()  # Clean up GPIO on exit