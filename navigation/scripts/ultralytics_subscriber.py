#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ultralytics_ros.msg import BoundingBox

def bounding_box_callback(msg):
    # Callback function to handle received bounding box messages
    # Process the bounding box information as needed
    rospy.loginfo("Received bounding box message:")
    rospy.loginfo("Center X: %f", msg.center_x)
    rospy.loginfo("Center Y: %f", msg.center_y)
    rospy.loginfo("Size X: %f", msg.size_x)
    rospy.loginfo("Size Y: %f", msg.size_y)
    rospy.loginfo("Class Name: %s", msg.class_name)
    rospy.loginfo("Confidence: %f", msg.confidence)
    rospy.loginfo("---------------------------")

def main():
    rospy.init_node("bounding_box_subscriber")
    
    # Subscribe to the bounding_box topic
    rospy.Subscriber("bounding_box", BoundingBox, bounding_box_callback)
    
    # Spin to keep the node running and handling messages
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
