#include <control.h>

static const double PI = atan(1.0) * 4.0;
static const double PI2 = 3.141592;
float speed(0.0); // Linear velocity (m/s)
float turn(0.0);  // Angular velocity (rad/s)
float kali_L(0.0);
float kali_A(0.0);
//==============================================================================
// Constructor
//==============================================================================

Control::Control(void)
{
    ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    ros::param::get("NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS);
    ros::param::get("NUMBER_OF_HEAD_SEGMENTS", NUMBER_OF_HEAD_JOINTS);
    ros::param::get("BODY_MAX_ROLL", BODY_MAX_ROLL);
    ros::param::get("BODY_MAX_PITCH", BODY_MAX_PITCH);
    ros::param::get("BODY_MAX_YAW", BODY_MAX_YAW);
    ros::param::get("HEAD_MAX_YAW", HEAD_MAX_YAW);
    ros::param::get("HEAD_MAX_PITCH", HEAD_MAX_PITCH);
    ros::param::get("STANDING_BODY_HEIGHT", STANDING_BODY_HEIGHT);
    ros::param::get("SERVOS", SERVOS);
    ros::param::get("MAX_BODY_ROLL_COMP", MAX_BODY_ROLL_COMP);
    ros::param::get("MAX_BODY_PITCH_COMP", MAX_BODY_PITCH_COMP);
    ros::param::get("COMPENSATE_INCREMENT", COMPENSATE_INCREMENT);
    ros::param::get("COMPENSATE_TO_WITHIN", COMPENSATE_TO_WITHIN);
    ros::param::get("MASTER_LOOP_RATE", MASTER_LOOP_RATE);
    ros::param::get("VELOCITY_DIVISION", VELOCITY_DIVISION);
    ros::param::get("MAX_METERS_PER_SEC", speed);
    ros::param::get("MAX_RADIANS_PER_SEC", turn);
    //Adit
    ros::param::get("LINEAR_A", kali_L);
    ros::param::get("ANGULAR_A", kali_A);
    
    current_time_odometry_ = ros::Time::now();
    last_time_odometry_ = ros::Time::now();
    current_time_cmd_vel_ = ros::Time::now();
    last_time_cmd_vel_ = ros::Time::now();
    // Find out how many servos/joints we have
    for (XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++)
    {
        servo_map_key_.push_back(it->first);
    }
    joint_state_.name.resize(servo_map_key_.size());
    joint_state_.position.resize(servo_map_key_.size());
    servo_names_.resize(servo_map_key_.size());
    servo_orientation_.resize(servo_map_key_.size());
    for (std::size_t i = 0; i < servo_map_key_.size(); i++)
    {
        ros::param::get(("/SERVOS/" + static_cast<std::string>(servo_map_key_[i]) + "/name"), servo_names_[i]);
        ros::param::get(("/SERVOS/" + static_cast<std::string>(servo_map_key_[i]) + "/sign"), servo_orientation_[i]);
    }
    prev_hex_state_ = false;
    hex_state_ = true;
    imu_init_stored_ = false;
    imu_override_.data = false;
    imu_roll_lowpass_ = 0.0;
    imu_pitch_lowpass_ = 0.0;
    imu_yaw_lowpass_ = 0.0;
    imu_roll_init_ = 0.0;
    imu_pitch_init_ = 0.0;
    odomOld.pose.pose.position.x = 0;
    odomOld.pose.pose.position.y = 0;
    odomOld.pose.pose.orientation.z = 0;

    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Control::cmd_velCallback, this);
    body_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>("/body_scalar", 1, &Control::bodyCallback, this);
    // head_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>("/head_scalar", 1, &Control::headCallback, this);
    head_sub_ = nh_.subscribe<geometry_msgs::Twist>("/head_Tws", 1, &Control::headCallback, this);
    state_sub_ = nh_.subscribe<std_msgs::Bool>("/state", 1, &Control::stateCallback, this);
    imu_override_sub_ = nh_.subscribe<std_msgs::Bool>("/imu/imu_override", 1, &Control::imuOverrideCallback, this);
    // imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 1, &Control::imuCallback, this);
    //subInitialPose = nh_.subscribe<geometry_msgs::PoseStamped>("/initial_2d", 1, &Control::set_initial_2d, this);
    //sub = n.subscribe("/tld_tracked_object", 20, &callback);
    subInitialPose = nh_.subscribe("/initial_2d", 1, &Control::set_initial_2d, this);
    // Topics we are publishing
    sounds_pub_ = nh_.advertise<hexapod_msgs::Sounds>("/sounds", 10);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom/calculated", 50);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/twist", 50);
    chatter_pub1 = nh_.advertise<std_msgs::Float32>("/chatter1", 1);
    chatter_pub2 = nh_.advertise<std_msgs::Float32>("/chatter2", 1);
    chatter_pub3 = nh_.advertise<std_msgs::Float32>("/chatter3", 1);

    // Send service request to the imu to re-calibrate
    // imu_calibrate_ = nh_.serviceClient<std_srvs::Empty>("/imu/calibrate");
    // imu_calibrate_.call(calibrate_);
}

void Control::set_initial_2d(const geometry_msgs::PoseStamped &rvizClick)
{
    odomOld.pose.pose.position.x = rvizClick.pose.position.x;
    odomOld.pose.pose.position.y = rvizClick.pose.position.y;
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseRecieved = true;
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setHexActiveState(bool state)
{
    hex_state_ = state;
}

bool Control::getHexActiveState(void)
{
    return hex_state_;
}

void Control::setPrevHexActiveState(bool state)
{
    prev_hex_state_ = state;
}

bool Control::getPrevHexActiveState(void)
{
    return prev_hex_state_;
}

//==============================================================================
// Odometry Publisher
//==============================================================================
void Control::publishOdometry(const geometry_msgs::Twist &gait_vel)
{
    // compute odometry in a typical way given the velocities of the robot

    // calculate time elapsed
    current_time_odometry_ = ros::Time::now();
    double dt = (current_time_odometry_ - last_time_odometry_).toSec();

    double vth = gait_vel.angular.z * kali_A;
    double delta_th = vth * dt;
    pose_th_ += delta_th;

    odomNew.pose.pose.orientation.z = delta_th + odomOld.pose.pose.orientation.z;

    double vx = gait_vel.linear.x * kali_L;
    double vy = gait_vel.linear.y * kali_L;
    //double delta_x = (vx * cos(pose_th_) - vy * sin(pose_th_)) * dt;
    //double delta_y = (vx * sin(pose_th_) + vy * cos(pose_th_)) * dt;
    double delta_x = (vx * cos(odomNew.pose.pose.orientation.z) - vy * sin(odomNew.pose.pose.orientation.z)) * dt;
    double delta_y = (vx * sin(odomNew.pose.pose.orientation.z) + vy * cos(odomNew.pose.pose.orientation.z)) * dt;
    // double delta_x = vx * dt;
    // double delta_y = vy  * dt;
    pose_x_ += delta_x;
    pose_y_ += delta_y;
    pose_th_ += delta_th;
    // msg.data = odom.pose.pose.orientation.x;
    

    //calculate
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + delta_x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + delta_y;
    odomNew.pose.pose.orientation.z = delta_th + odomOld.pose.pose.orientation.z;

    msg1.data=pose_x_;
    msg2.data=pose_y_;
    msg3.data=pose_th_;
    chatter_pub1.publish(msg1);
    chatter_pub2.publish(msg2);
    chatter_pub3.publish(msg3);

    //Prevent lockup from a single bad cycle
    if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z))
    {
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
    }
    //Make sure theta stays in the correct range

    // Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
    odomOld.header.stamp = odomNew.header.stamp;


    tf2::Quaternion q;
         
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odomNew.pose.pose.orientation.z);

    

    // first, we'll publish the transform over tf

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_odometry_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odomNew.pose.pose.position.x;
    odom_trans.transform.translation.y = odomNew.pose.pose.position.y;
    odom_trans.transform.translation.z = body_.position.z;
    odom_trans.transform.rotation = odom_quat;
    

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_odometry_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // set the position
    odom.pose.pose.position.x = odomNew.pose.pose.position.x;
    odom.pose.pose.position.y = odomNew.pose.pose.position.y;
    odom.pose.pose.position.z = body_.position.z;
    odom.pose.pose.orientation = odom_quat;

    // odom.pose.pose.orientation.x = q.x();
    // odom.pose.pose.orientation.y = q.y();
    // odom.pose.pose.orientation.z = odomNew.pose.pose.orientation.z; //q.z();
    // odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance[0] = 0.00001;          // x
    odom.pose.covariance[7] = 0.00001;          // y
    odom.pose.covariance[14] = 0.00001;         // z
    odom.pose.covariance[21] = 1000000000000.0; // rot x
    odom.pose.covariance[28] = 1000000000000.0; // rot y
    odom.pose.covariance[35] = 0.001;           // rot z

    // set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = odom.pose.covariance; // needed?

    odom_pub_.publish(odom);
    last_time_odometry_ = current_time_odometry_;

}



//==============================================================================
// Twist Publisher
//==============================================================================
void Control::publishTwist(const geometry_msgs::Twist &gait_vel)
{
    geometry_msgs::TwistWithCovarianceStamped twistStamped;
    twistStamped.header.stamp = ros::Time::now();
    twistStamped.header.frame_id = "odom";

    twistStamped.twist.twist.linear.x = gait_vel.linear.x;
    twistStamped.twist.twist.linear.y = gait_vel.linear.y;
    twistStamped.twist.twist.angular.z = gait_vel.angular.z;

    twistStamped.twist.covariance[0] = 0.00001;          // x
    twistStamped.twist.covariance[7] = 0.00001;          // y
    twistStamped.twist.covariance[14] = 0.00001;         // z
    twistStamped.twist.covariance[21] = 1000000000000.0; // rot x
    twistStamped.twist.covariance[28] = 1000000000000.0; // rot y
    twistStamped.twist.covariance[35] = 0.001;           // rot z

    twist_pub_.publish(twistStamped);
}

//==============================================================================
// Joint State Publisher
//==============================================================================
void Control::publishJointStates(const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::RPY &head, sensor_msgs::JointState *joint_state)
{
    joint_state->header.stamp = ros::Time::now();
    int i = 0;
    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].coxa;
        i++;
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].femur;
        i++;
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tibia;
        i++;
        if (NUMBER_OF_LEG_JOINTS == 4)
        {
            joint_state->name[i] = servo_names_[i];
            joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tarsus;
            i++;
        }
    }
    switch (NUMBER_OF_HEAD_JOINTS)
    {
    case 1:
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = head_.yaw;
        i++;
        break;
    case 2:
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = head_.yaw;
        i++;
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = head_.pitch;
        i++;
        break;
    default:
        break;
    }
    joint_state_pub_.publish(*joint_state);
}

//==============================================================================
// Topics we subscribe to
//==============================================================================
//==============================================================================
// cmd_vel callback
//==============================================================================

void Control::cmd_velCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg)
{
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

void Control::bodyCallback(const geometry_msgs::AccelStampedConstPtr &body_scalar_msg)
{
    ros::Time current_time = ros::Time::now();
    double time_delta = current_time.toSec() - body_scalar_msg->header.stamp.toSec();
    if (time_delta < 1.0) // Don't move if timestamp is stale over a second
    {
        if (imu_override_.data == true)
        {
            // To prevent violent motion changes the values are ran through a low pass filter
            body_.orientation.roll = (body_scalar_msg->accel.angular.x * BODY_MAX_ROLL) * 0.01 + (body_.orientation.roll * (1.0 - 0.01));
            body_.orientation.pitch = (body_scalar_msg->accel.angular.y * BODY_MAX_PITCH) * 0.01 + (body_.orientation.pitch * (1.0 - 0.01));
            body_.orientation.yaw = (body_scalar_msg->accel.angular.z * BODY_MAX_YAW) * 0.01 + (body_.orientation.yaw * (1.0 - 0.01));
            sounds_.auto_level = true;
            sounds_pub_.publish(sounds_);
            sounds_.auto_level = false;
        }
    }
}

//==============================================================================
// Pan head callback
//==============================================================================

// void Control::headCallback(const geometry_msgs::AccelStampedConstPtr &head_scalar_msg)
// {
//     ros::Time current_time = ros::Time::now();
//     double time_delta = current_time.toSec() - head_scalar_msg->header.stamp.toSec();
//     if (time_delta < 1.0) // Don't move if timestamp is stale over a second
//     {
//         head_.yaw = head_scalar_msg->accel.angular.z * HEAD_MAX_YAW;
//         head_.pitch = head_scalar_msg->accel.angular.y * HEAD_MAX_PITCH;
//         // sounds_.auto_level = true;
//         // sounds_pub_.publish(sounds_);
//         // sounds_.auto_level = false;
//     }
// }
//==============================================================================
// Pan head callback - full vector
//==============================================================================

void Control::headCallback(const geometry_msgs::TwistConstPtr &head_Tws_msg)
{
    
        head_.yaw = head_Tws_msg->linear.x * HEAD_MAX_YAW;
        head_.pitch = head_Tws_msg->linear.y * HEAD_MAX_PITCH;

        
    
}
//==============================================================================
// Active state callback - currently simple on/off - stand/sit
//==============================================================================

void Control::stateCallback(const std_msgs::BoolConstPtr &state_msg)
{
    if (state_msg->data == true)
    {
        if (getHexActiveState() == false)
        {
            // Activating hexapod
            body_.position.y = 0.0;
            body_.position.z = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            setHexActiveState(true);
            sounds_.stand = true;
            sounds_pub_.publish(sounds_);
            sounds_.stand = false;
        }
    }

    if (state_msg->data == false)
    {
        if (getHexActiveState() == true)
        {
            // Sit down hexapod
            body_.position.y = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            head_.yaw = 0.0;
            head_.pitch = 0.0;
            setHexActiveState(false);
            sounds_.shut_down = true;
            sounds_pub_.publish(sounds_);
            sounds_.shut_down = false;
        }
    }
}

//==============================================================================
// IMU override callback
//==============================================================================

void Control::imuOverrideCallback(const std_msgs::BoolConstPtr &imu_override_msg)
{
    imu_override_.data = imu_override_msg->data;
}

//==============================================================================
// IMU callback to auto-level body if on non level ground
//==============================================================================

// void Control::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
// {
//     if (imu_override_.data == false)
//     {
//         const geometry_msgs::Vector3 &lin_acc = imu_msg->linear_acceleration;

//         if (imu_init_stored_ == false)
//         {
//             imu_roll_init_ = atan2(lin_acc.x, sqrt(lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z));
//             imu_pitch_init_ = -atan2(lin_acc.y, lin_acc.z);
//             imu_pitch_init_ = (imu_pitch_init_ >= 0.0) ? (PI - imu_pitch_init_) : (-imu_pitch_init_ - PI);
//             imu_init_stored_ = true;
//         }

//         // low-pass filter to smooth out noise
//         imu_roll_lowpass_ = lin_acc.x * 0.01 + (imu_roll_lowpass_ * (1.0 - 0.01));
//         imu_pitch_lowpass_ = lin_acc.y * 0.01 + (imu_pitch_lowpass_ * (1.0 - 0.01));
//         imu_yaw_lowpass_ = lin_acc.z * 0.01 + (imu_yaw_lowpass_ * (1.0 - 0.01));

//         double imu_roll = atan2(imu_roll_lowpass_, sqrt(imu_pitch_lowpass_ * imu_pitch_lowpass_ + imu_yaw_lowpass_ * imu_yaw_lowpass_));
//         double imu_pitch = -atan2(imu_pitch_lowpass_, imu_yaw_lowpass_);
//         imu_pitch = (imu_pitch >= 0.0) ? (PI - imu_pitch) : (-imu_pitch - PI);

//         double imu_roll_delta = imu_roll_init_ - imu_roll;
//         double imu_pitch_delta = imu_pitch_init_ - imu_pitch;

//         if ((std::abs(imu_roll_delta) > MAX_BODY_ROLL_COMP) || (std::abs(imu_pitch_delta) > MAX_BODY_PITCH_COMP))
//         {
//             // sounds_.auto_level = true;
//             // sounds_pub_.publish(sounds_);
//             // sounds_.auto_level = false;
//         }

//         if (imu_roll_delta < -COMPENSATE_TO_WITHIN)
//         {
//             if (body_.orientation.roll < MAX_BODY_ROLL_COMP)
//             {
//                 body_.orientation.roll = body_.orientation.roll + COMPENSATE_INCREMENT;
//             }
//         }

//         if (imu_roll_delta > COMPENSATE_TO_WITHIN)
//         {
//             if (body_.orientation.roll > -MAX_BODY_ROLL_COMP)
//             {
//                 body_.orientation.roll = body_.orientation.roll - COMPENSATE_INCREMENT;
//             }
//         }

//         if (imu_pitch_delta < -COMPENSATE_TO_WITHIN)
//         {
//             if (body_.orientation.pitch < MAX_BODY_PITCH_COMP)
//             {
//                 body_.orientation.pitch = body_.orientation.pitch + COMPENSATE_INCREMENT;
//             }
//         }

//         if (imu_pitch_delta > COMPENSATE_TO_WITHIN)
//         {
//             if (body_.orientation.pitch > -MAX_BODY_PITCH_COMP)
//             {
//                 body_.orientation.pitch = body_.orientation.pitch - COMPENSATE_INCREMENT;
//             }
//         }
//     }
// }

//==============================================================================
// Partitions up the cmd_vel to the speed of the loop rate
//==============================================================================

void Control::partitionCmd_vel(geometry_msgs::Twist *cmd_vel)
{
    // Instead of getting delta time we are calculating with a static division
    double dt = VELOCITY_DIVISION;

    double delta_th = cmd_vel_incoming_.angular.z * dt;
    double delta_x = (cmd_vel_incoming_.linear.x * cos(delta_th) - cmd_vel_incoming_.linear.y * sin(delta_th)) * dt;
    double delta_y = (cmd_vel_incoming_.linear.x * sin(delta_th) + cmd_vel_incoming_.linear.y * cos(delta_th)) * dt;
    cmd_vel->linear.x = delta_x * speed;
    cmd_vel->linear.y = delta_y * speed;
    cmd_vel->angular.z = delta_th * turn;
}
