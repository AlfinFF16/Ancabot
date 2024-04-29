#!/usr/bin/env python3

import rospy
import os
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool

# Define GPIO pin for the push button
BUTTON_PIN = 17

package = "hexapod_controller"
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
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Subscribe to button topic
    GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

    rospy.loginfo("Button controller node ready...")

    rospy.spin()

    # Clean up GPIO
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
