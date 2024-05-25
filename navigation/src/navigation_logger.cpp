#include "ros/ros.h"
#include "ultralytics_ros/BoundingBox.h"
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <map>

// Define the file name for CSV
const std::string FILENAME = "/home/jetson/ancabot_ws/src/navigation/logger/sensor_data.csv";

// Define a function to write data to CSV file
void writeDataToCSV(const std::vector<float>& ping, char cmd) {
    std::ofstream outfile;
    outfile.open(FILENAME, std::ios_base::app); // Append mode
    if (outfile.is_open()) {
        // Write timestamp
        outfile << ros::Time::now().toSec() << ",";
        // Write sensor data
        for (const auto& val : ping) {
            outfile << val << ",";
        }
        // Write move binding character
        outfile << cmd << std::endl;
        outfile.close();
    } else {
        ROS_ERROR("Unable to open file: %s", FILENAME.c_str());
    }
}

void tofdistancesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::vector<float> ping(8, 0.0f); // Initialize ping vector with zeros
    for (int i = 0; i < 4; i++) {
        ping[i] = msg->data[i];
    }
    ROS_INFO("ToF Reading: front=%f back=%f left=%f right=%f", ping[0], ping[1], ping[2], ping[3]);
    // Write data to CSV
    writeDataToCSV(ping, ' ');
}

void eulerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::vector<float> ping(8, 0.0f); // Initialize ping vector with zeros
    ping[4] = std::abs(msg->pose.position.x); // roll orientation
    ping[5] = std::abs(msg->pose.position.y); // pitch orientation
    ping[6] = std::abs(msg->pose.position.z); // yaw orientation

    ROS_INFO("IMU Orientation: roll=%f pitch=%f yaw=%f", ping[4], ping[5], ping[6]);
    // Write data to CSV
    writeDataToCSV(ping, ' ');
}

void boundingBoxCallback(const ultralytics_ros::BoundingBox::ConstPtr& msg) {
    std::vector<float> ping(8, 0.0f); // Initialize ping vector with zeros
    if (msg->class_name == "tensor(1.)" && msg->confidence >= 0.7) {
        ping[7] = msg->center_x;
        ROS_INFO("Center Coordinate of korban: x = %f; y = %f", msg->center_x, msg->center_y);
    } else {
        ROS_INFO("Korban isn't detected yet");
    }
    // Write data to CSV
    writeDataToCSV(ping, ' ');
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Extracting values from the received message
    float x = msg->linear.x;
    float y = msg->linear.y;
    float z = msg->linear.z;
    float th = msg->angular.z;

    // Define move binding character
    char cmd = ' '; // Default value

    // Define move bindings
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

    // Find matching move binding
    for (auto& binding : moveBindings) {
        if (x == binding.second[0] && y == binding.second[1] && z == binding.second[2] && th == binding.second[3]) {
            cmd = binding.first;
            break;
        }
    }

    // If a match is found, publish the corresponding character
    if (cmd != ' ') {
        ROS_INFO("Correspond movement command: %c", cmd);
    } else {
        ROS_WARN("No matching command found for the received twist message");
    }

    std::vector<float> ping(8, 0.0f); // Initialize ping vector with zeros
    // Write data to CSV
    writeDataToCSV(ping, cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_debugging");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber euler_sub = nh.subscribe("/euler_topic", 10, eulerCallback);
    ros::Subscriber tof_sub = nh.subscribe("/tof_distances", 10, tofdistancesCallback);
    ros::Subscriber ultralytics_sub = nh.subscribe("/bounding_box", 10, boundingBoxCallback);

    ros::spin();

    return 0;
}
