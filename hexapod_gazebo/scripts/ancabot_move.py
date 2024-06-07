#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# List of joint names for the hexapod
joints = [
    'pan_joint', 'tilt_joint',
    'coxa_joint_RR', 'coxa_joint_RM', 'coxa_joint_RF',
    'coxa_joint_LR', 'coxa_joint_LM', 'coxa_joint_LF',
    'femur_joint_RR', 'femur_joint_RM', 'femur_joint_RF',
    'femur_joint_LR', 'femur_joint_LM', 'femur_joint_LF',
    'tibia_joint_RR', 'tibia_joint_RM', 'tibia_joint_RF',
    'tibia_joint_LR', 'tibia_joint_LM', 'tibia_joint_LF'
]

# Publishers for each joint
publishers = {}

def initialize_publishers():
    global publishers
    for joint in joints:
        topic_name = f'/Ancabot/{joint}_position_controller/command'
        publishers[joint] = rospy.Publisher(topic_name, Float64, queue_size=10)

def joint_states_callback(msg):
    joint_positions = dict(zip(msg.name, msg.position))
    for joint in joints:
        if joint in joint_positions:
            position = joint_positions[joint]
            publishers[joint].publish(Float64(position))

if __name__ == '__main__':
    try:
        rospy.init_node('hexapod_joint_state_subscriber', anonymous=True)
        initialize_publishers()
        rospy.Subscriber('/joint_states', JointState, joint_states_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
