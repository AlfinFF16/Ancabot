#include <ros/ros.h>
#include <control.h>
#include <gait.h>
#include <ik.h>
#include <servo_driver.h>

#include <geometry_msgs/PoseStamped.h>

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_controller");

    // Create class objects
    Control control;
    Gait gait;
    Ik ik;
    ServoDriver servoDriver;

    // Establish initial leg positions for default pose in robot publisher
    gait.gaitCycle(control.cmd_vel_, &control.feet_, &control.gait_vel_);
    ik.calculateIK(control.feet_, control.body_, &control.legs_);
    control.publishJointStates(control.legs_, control.head_, &control.joint_state_);
    control.publishOdometry(control.gait_vel_);
    control.publishTwist(control.gait_vel_);

    ros::Time current_time_, last_time_;
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    ros::AsyncSpinner spinner(2); // Using 2 threads
    spinner.start();
    ros::Rate loop_rate(300);

    while (ros::ok())
    {
        current_time_ = ros::Time::now();
        double dt = (current_time_ - last_time_).toSec();
        // Divide cmd_vel by the loop rate to get appropriate velocities for gait period
        control.partitionCmd_vel(&control.cmd_vel_);

        // Set timeout duration
        ros::Duration timeout_duration(10.0); // Example timeout duration of 10 seconds

        // Hexapod standing up.
        while (control.body_.position.z < control.STANDING_BODY_HEIGHT && ros::ok())
        {
            control.body_.position.z = control.body_.position.z + 0.001; // 1 mm increment

            // IK solver for legs and body orientation
            ik.calculateIK(control.feet_, control.body_, &control.legs_);

            // Commit new positions and broadcast over USB2AX as well as jointStates
            control.publishJointStates(control.legs_, control.head_, &control.joint_state_);
            servoDriver.transmitServoPositions(control.joint_state_);
            control.publishOdometry(control.gait_vel_);
            control.publishTwist(control.gait_vel_);

            // Add a check to prevent infinite loop
            if (ros::Time::now() - current_time_ > timeout_duration) {
                ROS_ERROR("Timeout occurred during standing up process.");
                break;
            }
        }

        // Hexapod sitting down.
        while (control.body_.position.z > 0 && ros::ok())
        {
            control.body_.position.z = control.body_.position.z - 0.001; // 1 mm increment

            // Gait Sequencer called to make sure we are on all six feet
            gait.gaitCycle(control.cmd_vel_, &control.feet_, &control.gait_vel_);

            // IK solver for legs and body orientation
            ik.calculateIK(control.feet_, control.body_, &control.legs_);

            // Commit new positions and broadcast over USB2AX as well as jointStates
            control.publishJointStates(control.legs_, control.head_, &control.joint_state_);
            servoDriver.transmitServoPositions(control.joint_state_);
            control.publishOdometry(control.gait_vel_);
            control.publishTwist(control.gait_vel_);

            // Add a check to prevent infinite loop
            if (ros::Time::now() - current_time_ > timeout_duration) {
                ROS_ERROR("Timeout occurred during sitting down process.");
                break;
            }
        }

        loop_rate.sleep();
        last_time_ = current_time_;
    }

    return 0;
}
