#!/usr/bin/python

import time
import VL53L0X
import rospy
from std_msgs.msg import Int32


def publish_distance(distance):
    # Publish the distance
    msg = Int32(data=distance)
    pub.publish(msg)


# Initialize ROS node
rospy.init_node('tof_publisher', anonymous=True)
pub = rospy.Publisher('tof_0', Int32, queue_size=10)

# Create VL53L0X object
sensor = VL53L0X.VL53L0X(i2c_bus=0, i2c_address=0x29)
sensor.open()
sensor.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

# Get the timing from the first sensor if available
timing = sensor.get_timing()
if timing < 100000:
    timing = 100000    
rospy.loginfo("Timing %d ms" % (timing / 1000))

try:
    while not rospy.is_shutdown():
        distance = sensor.get_distance()
        if distance > 0:
            publish_distance(distance)
            rospy.loginfo("Distance: %d mm" % (distance))
        else:
            distance = 0
            publish_distance(distance)
            rospy.logerr("Error: Failed to get distance from the sensor")
        time.sleep(timing / 1000000.00)

except rospy.ROSInterruptException:
    rospy.loginfo("ROS node interrupted")
finally:
    # Stop ranging for the sensor before exiting
    sensor.stop_ranging()
    sensor.close()
    rospy.loginfo("Sensor stopped and closed")