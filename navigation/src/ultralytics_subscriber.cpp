#include "ros/ros.h"
#include "ultralytics_ros/BoundingBox.h"

float x_center = 0;
float y_center = 0;

void boundingBoxCallback(const ultralytics_ros::BoundingBox::ConstPtr& msg)
{
    if (msg->class_name == "tensor(1.)" && msg->confidence >= 0.7)
    {
        x_center = msg->center_x;
        y_center = msg->center_y;

        ROS_INFO("Center Coordinate of korban: x = %f; y = %f", x_center, y_center);
    }
    else
    {
        x_center = 0;
        y_center = 0;

        ROS_INFO("Korban isn't detected yet");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bounding_box_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("bounding_box", 10, boundingBoxCallback);
    ros::spin();

    return 0;
}
