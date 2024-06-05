#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('/pexod/vel_cmd', Float32, queue_size=1)
    rospy.init_node('hexapod_mover_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    position = Float32()
    position.data = 0.7
    while not rospy.is_shutdown():
        rospy.loginfo("Moving Hexapod to position =="+str(position))
        pub.publish(position)
        rate.sleep()
        position.data *= -1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass