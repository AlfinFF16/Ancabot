#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO
import time

# Define GPIO pins for the sensors
GPIO_SIG1 = 27  # GPIO pin for the first sensor
GPIO_SIG2 = 22  # GPIO pin for the second sensor

def measured_distance(GPIO_SIG):
    # Set up GPIO_SIG as output and send a trigger signal
    GPIO.setup(GPIO_SIG, GPIO.OUT)
    GPIO.output(GPIO_SIG, GPIO.LOW)
    time.sleep(0.000002)  # Delay to ensure the trigger is clear

    GPIO.output(GPIO_SIG, GPIO.HIGH)
    time.sleep(0.000010)  # Trigger signal duration
    GPIO.output(GPIO_SIG, GPIO.LOW)

    # Set up GPIO_SIG as input to read the echo signal
    GPIO.setup(GPIO_SIG, GPIO.IN)

    # Record the start time
    starttime = time.time()
    while GPIO.input(GPIO_SIG) == GPIO.LOW:
        starttime = time.time()

    # Record the end time
    endtime = time.time()
    while GPIO.input(GPIO_SIG) == GPIO.HIGH:
        endtime = time.time()

    # Calculate the duration
    duration = endtime - starttime

    # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s
    distance = (duration * 34000) / 2

    return round(distance, 2)  # Round the distance to two decimal places

def publish_distances():
    # Initialize ROS node and publisher
    rospy.init_node('ultrasonic_sensor_publisher', anonymous=True)
    pub = rospy.Publisher('ping_distances', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz - Adjust the rate as needed

    while not rospy.is_shutdown():
        # Measure distance for sensor 1
        try:
            distance1 = measured_distance(GPIO_SIG1)
        except:
            rospy.logerr("Error reading distance from sensor 1")
            distance1 = 0.0

        # Measure distance for sensor 2
        try:
            distance2 = measured_distance(GPIO_SIG2)
        except:
            rospy.logerr("Error reading distance from sensor 2")
            distance2 = 0.0

        # Log distances
        rospy.loginfo("Ping 1: %.2f cm, Ping 2: %.2f cm" % (distance1, distance2))

        # Publish the distances
        msg = Float64MultiArray(data=[distance1, distance2])
        pub.publish(msg)
        rate.sleep()

if __name__ == '_main_':
    try:
        # Set GPIO mode and start publishing distances
        GPIO.setmode(GPIO.BCM)
        publish_distances()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Cleanup GPIO before exiting
        GPIO.cleanup()