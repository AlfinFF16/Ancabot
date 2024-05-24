#include "ros/ros.h"
#include "ultralytics_ros/BoundingBox.h"
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

float ping[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void tofdistancesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < 4; i++)
    {
        ping[i] = msg->data[i];
    }
    ROS_INFO("ToF Reading: front=%f back=%f left=%f right=%f", ping[0], ping[1], ping[2], ping[3]);
}

void eulerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ping[4] = std::abs(msg->pose.position.x) * (180.0 / M_PI); // roll orientation
    ping[5] = std::abs(msg->pose.position.y) * (180.0 / M_PI); // pitch orientation
    ping[6] = std::abs(msg->pose.position.z) * (180.0 / M_PI); // yaw orientation

    ROS_INFO("IMU Orientation: roll=%f pitch=%f yaw=%f", ping[4], ping[5], ping[6]);
}

void boundingBoxCallback(const ultralytics_ros::BoundingBox::ConstPtr& msg)
{
    if (msg->class_name == "tensor(1.)" && msg->confidence >= 0.7)
    {
        ping[7] = msg->center_x;
        ROS_INFO("Center Coordinate of korban: x = %f; y = %f", msg->center_x, msg->center_y);
    }
    else
    {
        ping[7] = 0;
        ROS_INFO("Korban isn't detected yet");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_debugging");
    ros::NodeHandle nh;

    ros::Subscriber euler_sub = nh.subscribe("/euler_topic", 10, eulerCallback);
    ros::Subscriber tof_sub = nh.subscribe("/tof_distances", 10, tofdistancesCallback);
    ros::Subscriber ultralytics_sub = nh.subscribe("/bounding_box", 10, boundingBoxCallback);

    ros::spin();

    return 0;
}
