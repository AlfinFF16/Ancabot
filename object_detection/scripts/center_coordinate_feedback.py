#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from object_detection.msg import centerCoordinate

from ultralytics import YOLO

class ObjectDetectorROS:
    def __init__(self):
        rospy.init_node('object_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("video_frames", Image, self.image_callback)
        # Initialize the YOLO model
        self.model = YOLO("weights/krsri24.pt")
        self.classNames = ["dummy", "korban"]
        self.pub = rospy.Publisher('object_center', centerCoordinate, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("Error converting ROS image message to OpenCV image: %s", e)
            return

        # Perform object detection
        results = self.model(cv_image, stream=True)

        # Annotate the image with bounding boxes and class names
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                confidence = math.ceil((box.conf[0]*100))/100
                rospy.loginfo("Confidence: %f", confidence)

                cls = int(box.cls[0])
                class_name = self.classNames[cls]
                rospy.loginfo("Class name: %s", class_name)

                if class_name == "korban":
                    # Calculate center coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Publish center coordinates
                    center_point = centerCoordinate()
                    center_point.x = center_x
                    center_point.y = center_y
                    self.pub.publish(center_point)

                    rospy.loginfo("coordinate of 'korban': x={}, y={}".format(center_x, center_y))

if __name__ == '__main__':
    try:
        obj_detector = ObjectDetectorROS()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")