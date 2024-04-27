#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from object_detection.msg import centerCoordinate
import os

from ultralytics import YOLO

class ObjectDetectorROS:
    def __init__(self):
        rospy.init_node('object_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("video_frames", Image, self.image_callback)
        # Initialize the YOLO model
        script_dir = os.path.dirname(os.path.abspath(/home/jetson/ancabot/src/Ancabot/object_detection/scripts))
        weights_path = os.path.join(script_dir, "weights/krsri24.pt")
        self.model = YOLO(weights_path)
        self.classNames = ["dummy", "korban"]
        self.pub = rospy.Publisher('object_center', Point, queue_size=10)

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
                    centerCoordinate.x = center_x
                    centerCoordinate.y = center_y
                    self.pub.publish(center_point)

                # Convert cv_image to UMat object
                cv_image_um = cv2.UMat(cv_image)

                try:
                    # Draw rectangle
                    cv2.rectangle(cv_image_um, (x1, y1), (x2, y2), (255, 0, 255), 3)
                except cv2.error as e:
                    rospy.logerr("Error drawing rectangle: %s", e)
                    return

                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(cv_image_um, class_name, org, font, fontScale, color, thickness)

        # Resize the image for display
        cv_image_resized = cv_image_um.get()

        # Display the camera feed and annotated image side by side
        cv2.imshow('Camera Feed with Object Detection', cv_image_resized)
        cv2.waitKey(3)

if __name__ == '__main__':
    try:
        obj_detector = ObjectDetectorROS()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()