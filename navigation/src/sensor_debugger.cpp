#include "ros/ros.h"
#include "ultralytics_ros/BoundingBox.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Twist twist;

float ping[8] = {0, 0, 0, 0, 0, 0, 0, 0};

std::map<char, std::vector<float>> moveBindings{
    //Moving and Rotating
    {'q', {1, 0, 0, 1}},
    {'w', {1, 0, 0, 0}},
    {'e', {1, 0, 0, -1}},
    {'a', {0, 0, 0, 1}},
    {'s', {0, 0, 0, 0}},
    {'d', {0, 0, 0, -1}},
    {'z', {-1, 0, 0, -1}},
    {'x', {-1, 0, 0, 0}},
    {'c', {-1, 0, 0, 1}},
    //Holomonic Move
    {'Q', {1, -1, 0, 0}},
    {'W', {1, 0, 0, 0}},
    {'E', {1, 1, 0, 0}},
    {'A', {0, -1, 0, 0}},
    {'S', {0, 0, 0, 0}},
    {'D', {0, 1, 0, 0}},
    {'Z', {-1, -1, 0, 0}},
    {'X', {-1, 0, 0, 0}},
    {'C', {-1, 1, 0, 0}}
};

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Extracting values from the received message
    float x = msg->linear.x;
    float y = msg->linear.y;
    float z = msg->linear.z;
    float th = msg->angular.z;

    // Iterate over moveBindings to find a match
    char cmd = ' '; // Default value
    for (auto& binding : moveBindings) {
        if (x == binding.second[0] && y == binding.second[1] && z == binding.second[2] && th == binding.second[3]) {
            cmd = binding.first;
            break;
        }
    }

    ROS_INFO("Correspond movement command: %c", cmd);
}

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
    ping[4] = std::abs(msg->pose.position.x); // roll orientation
    ping[5] = std::abs(msg->pose.position.y); // pitch orientation
    ping[6] = std::abs(msg->pose.position.z); // yaw orientation

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

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber euler_sub = nh.subscribe("/euler_topic", 10, eulerCallback);
    ros::Subscriber tof_sub = nh.subscribe("/tof_distances", 10, tofdistancesCallback);
    ros::Subscriber ultralytics_sub = nh.subscribe("/bounding_box", 10, boundingBoxCallback);

    ros::spin();

    return 0;
}
