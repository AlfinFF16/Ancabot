#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ultralytics_ros/BoundingBox.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

float ping[8]={0,0,0,0,0,0,0,0};
// front tof, back tof, left tof, right tof, imu roll, imu pitch, mu yaw, center pose of detected object

void tofdistancesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  for (int i=0;i<4;i++){
    ping[i]=msg->data[i];
  }
  ROS_INFO("ToF Reading: front=%f back=%f left=%f  right=%f", ping[0], ping[1], ping[2], ping[3]);
}

void eulerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Extract Euler angles from the received message
  ping[4] = std::abs(msg->pose.position.x);     // roll orientation
  ping[5] = std::abs(msg->pose.position.y);     // pitch orientation
  ping[6] = std::abs(msg->pose.position.z);     // yaw orientation

  ROS_INFO("IMU Orientation: roll=%f pitch=%f yaw=%f", ping[4], ping[5], ping[6]);
}

void boundingBoxCallback(const ultralytics_ros::BoundingBox::ConstPtr& msg)
{
    float x_center = 0;
    float y_center = 0;

    if (msg->class_name == "tensor(1.)" && msg->confidence >= 0.7)
    {
        x_center = msg->center_x;
        y_center = msg->center_y;
    }
    else
    {
        x_center = 0;
        y_center = 0;
    }

    ping[7] = x_center;

    ROS_INFO("Center Coordinate of korban: x = %f; y = %f", x_center, y_center);
}

float xaa[5],yaa[5],xas[5];
bool ff1,ff2,ff3;
void chatter1Callback(const std_msgs::Float32& msg)
{
  xaa[0]=msg.data;
  // ROS_INFO("I heard: [%f]", xaa[0]);
  if(ff1==false){ yaa[0]=xaa[0]; ff1=true;}
}

void chatter2Callback(const std_msgs::Float32& msg)
{
  xaa[1]=msg.data;
  // ROS_INFO("I heard: [%f]", xaa[1]);
  if(ff2==false){ yaa[1]=xaa[1];ff2=true;}
}

void chatter3Callback(const std_msgs::Float32& msg)
{
  xaa[2]=msg.data;
  // ROS_INFO("I heard: [%f]", xaa[2]);
  if(ff3==false){ yaa[2]=xaa[2];ff3=true;}
}

int flag1=1;

// Map for movement keys
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
    {'C', {-1, 1, 0, 0}}};

//step
char a_gerak[]  ={'d','w','a','w','s','s','x','d','w','w','w','A','w','d','w','s','x','x'};
// step            0   1   2   3   4   5   6   7   8   9  10   11  12  13  14  15  16  17

// Pengondisian step dan batas gerakan
std::map<int, std::vector<float>> step{
  // {step, {Tof_depan, Tof_belakang, Tof_kiri, Tof_kanan, Imu Yaw, X Coord of Detected Object, Gripper (lifter), Gripper (gripper), Speed, Turn}}
  // gripper: teleop 'o' --> {0,0}; teleop 'p' --> {-2,0}; teleop 'l' --> {0,-1}; teleop ';' --> {-1,-1}
  {0,   {500,0,100,100,0,0,90,0,      -2,0,1,1}},   // keluar dari home
  {1,   {0,520,220,0,0,0,0,0,         -2,0,2,1}},   // menuju zona K1
  {2,   {250,125,470,500,0,0,0,990,   -2,0,1,1}},   // berotasi hingga gripper sejajar K1
  {3,   {150,190,0,0,0,0,0,0,         0,-1,1,1}},   // mendekati K1 (gripper diturunkan dan terbuka) 
  {4,   {140,0,0,0,0,0,0,0,            0,0,1,1}},   // di posisi K1 dan gripper men-grip korban
  {5,   {140,0,0,0,0,0,0,0,           -2,0,1,1}},   // gripper dengan korban diangkat kembali
  {6,   {0,100,0,0,0,0,0,0,           -2,0,1,1}},   // keluar dari zona K1 
  {7,   {0,500,300,150,0,0,90,0,      -2,0,1,1}},   // berotasi sejajar jalur utama
  {8,   {0,580,0,0,0,0,0,0,           -2,0,2,1}},   // bergerak maju hingga ke R1 (Jalan Retak)
  {9,   {0,0,0,0,0,10,0,0,            -2,0,2,1}},   // bergerak maju di R1 (Jalan Retak)
  {10,  {300,0,0,0,0,0,0,0,           -2,0,2,1}},   // bergerak maju melewati R2 (Turunan) dan R3 (Bebatuan) - Menuju SZ1
  {11,  {0,0,70,0,0,0,0,0,            -2,0,1,1}},   // menyamping ke kiri sebelum ke SZ1
  {12,  {100,0,400,0,0,0,0,0,         -2,0,1,1}},   // maju dan bersiap ke SZ1
  {13,  {0,0,120,0,0,0,175,0,         -2,0,1,1}},   // berotasi hingga sejajar dengan SZ1
  {14,  {220,500,0,0,0,0,0,0,         -2,0,1,1}},   // mendekat pada SZ1
  {15,  {100,0,0,0,0,0,0,0,           0,-1,1,1}},   // menurunkan K1 di SZ1
  {16,  {0,500,0,0,0,0,0,0,           0,-1,1,1}},   // mundur dengan gripper masih terbuka (handling korban terangkat lagi)
  {17,  {0,430,0,0,0,0,0,0,           -2,0,1,1}},   // mundur hingga tegak lurus K2
};

std::map<int, std::vector<bool>> _f_{
  // komparator (0)(sensor>=batas) (1)(Sensor<=batas) (index 0 - 7)
  // uneven = (0, 1) && normal = (0, 0) (index 9 - 10) 
  {0,   {0,0,1,1,0,0,0,0, 1,0,0}},
  {1,   {0,0,0,0,0,0,0,0, 1,0,0}},
  {2,   {1,1,0,0,0,0,0,0, 1,0,0}}, 
  {3,   {1,0,0,0,0,0,0,0, 1,0,0}},
  {4,   {1,0,0,0,0,0,0,0, 1,0,0}},
  {5,   {0,0,0,0,0,0,0,0, 1,0,0}},
  {6,   {0,1,0,0,0,0,0,0, 1,0,0}},
  {7,   {0,0,1,1,0,0,0,0, 1,0,0}},
  {8,   {0,0,0,0,0,0,0,0, 1,0,0}},
  {9,   {0,0,0,0,0,0,0,0, 1,0,1}},
  {10,  {1,0,0,0,0,0,0,0, 1,0,0}},
  {11,  {0,0,1,0,0,0,0,0, 1,0,0}},
  {12,  {1,0,0,0,0,0,0,0, 1,0,0}},
  {13,  {0,0,1,0,0,0,0,0, 1,0,0}},
  {14,  {1,0,0,0,0,0,0,0, 1,0,0}},
  {15,  {1,0,0,0,0,0,0,0, 1,0,0}},
  {16,  {0,1,0,0,0,0,0,0, 1,0,0}},
  {17,  {0,1,0,0,0,0,0,0, 1,0,0}},
};

// Init variables
float speed(1);                                                     // Linear velocity (m/s)
float turn(1);                                                      // Angular velocity (rad/s)
float x(0), y(0), z(0), xa(0), ya(0), za(0), xb(0), yb(0), th(0);   // Forward/backward/neutral direction vars
char key(' ');
int offset(15);
bool isAvoidanceActive = false;
int currentStep = 0;
geometry_msgs::Twist twist;
geometry_msgs::Twist head_Tws;
std_msgs::Bool imu_override_;
std_msgs::Bool leg_height_;
std_msgs::Bool state_;
std_msgs::Int32 Led_;

void avoidance(){
  
  if (ping[0] <= 90 || ping[1] <=40 || ping[2] <= 30 || ping[3] <= 30) {
    isAvoidanceActive = true;
    if(ping[3] <= 30){
    //gerakan ke kiri
    twist.linear.x = 0;
    twist.linear.y = -0.5;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
 
  }
  if(ping[0] <= 70 && xb == -2){
    //gerakan mundur
    twist.linear.x = -0.5;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

  }
  if(ping[0] <= 70 && xb == 0 && yb == 0){
    //gerakan mundur
    twist.linear.x = -0.5;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

  }
    if(ping[1] <= 40){
    //gerakan maju
    twist.linear.x = 0.2;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

  }
  if(ping[2] <= 30){
    //gerakan ke kanan
    twist.linear.x = 0;
    twist.linear.y = 0.5;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
  } 
  }else {
    isAvoidanceActive = false;
  }
}

bool pilih;
bool diffOrient;
void kontrol(char arah_, int step_){
  
  if (isAvoidanceActive) {
    return;
  }

  if (step_ == 1)
  {
    // Conditioning for robot first orientation
    if(ping[3] > ping[2])         // if the right's measurement greater than left's
    {
      key = arah_;
      diffOrient = false;
    }
    else if(ping[3] > ping[2])    // if the right's measurement smaller than left's
    {
      key = 'a';
      diffOrient = true;
    }
  }
  else
  {
    key = arah_;
  }

  int batas[8];
  if (step.count(step_) == 1)
    {
      for(int a=0;a<8;a++){
        batas[a]=step[step_][a];
        if (diffOrient)
        {
          if (step_ == 13)
          {
            batas[6] = 5;
          }
        }
      }
      // currentStep = step_;
      xb=step[step_][8];
      yb=step[step_][9];
      speed=step[step_][10];
      turn=step[step_][11];
    }

  bool flag_[8];
  if (_f_.count(step_) == 1)
    {
      for(int a=0;a<8;a++){
        flag_[a]=_f_[step_][a];
        if (diffOrient)
        {
          if (step_ == 7)
          {
            flag_[6] = 1;
          }
          else if (step_ == 13)
          {
            flag_[6] = 1;
          }
        }
      }
    pilih=_f_[step_][8];
    imu_override_.data = _f_[step_][9];
    leg_height_.data = _f_[step_][10];
    }

  // Trying to stabilize the orientation of robot's body
  if (moveBindings.count(key) == 1 && (step_ == 10))
  {
    float desired_yaw = 90.0;
    float yaw_difference = desired_yaw - ping[6];
    if (std::abs(yaw_difference) > 5.0) 
    {
      // If the yaw difference is greater than 5 degrees, rotate until desired orientation is reached
      twist.angular.z = (yaw_difference > 0) ? 0.5 : -0.5; // Adjust the angular velocity as needed
      // Keep linear velocities zero
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.linear.z = 0;
    }
    else
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];
      imu_override_.data = false;
           
      ROS_INFO("\rCurrent: speed %f   | turn %f | Last command: %c   ", speed, turn, key);
    }
  }
  else
  {
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];
      imu_override_.data = false;
           
      ROS_INFO("\rCurrent: speed %f   | turn %f | Last command: %c   ", speed, turn, key);
    }
  }

    // Update the Twist message
    twist.linear.x = x * speed *0.5;
    twist.linear.y = y * speed* 0.5;
    twist.linear.z = z * speed *0.5;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn *0.5;

    head_Tws.linear.x = xb * 0.625 ; //lifter
    head_Tws.linear.y = yb * 1.1 ; //gripper

    state_.data = true;
    Led_.data=2;
    
  
    ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d", batas[0], batas[1], batas[2], batas[3], batas[4], batas[5], batas[6], batas[7]);
    ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f",ping[0],ping[1],ping[2],ping[3],ping[4],ping[5],ping[6],ping[7]);
    ROS_INFO("%d, %d, %d, %d",flag_[0],flag_[1],flag_[2],flag_[3]);


    bool s[8]={false,false,false,false,false,false};

  if(pilih==true){
    for (int a=0; a<8; a++){
    for (int a = 0; a < 8; a++) {
        if (flag_[a] == true) {
            if (ping[a] <= (batas[a] + offset) || ping[a] <= (batas[a] - offset)) {
                s[a] = true;
            } else {
                s[a] = false;
            }
        } else {
            if (ping[a] >= (batas[a] + offset) || ping[a] >= (batas[a] - offset)) {
                s[a] = true;
            } else {
                s[a] = false;
            }
        }
        yaa[a] = xaa[a];
      }
    }
  }
  else{
    for (int a=0; a < 8; a++){
      xas[a]=xaa[a]-yaa[a];
      if(flag_[a]==true){
        if(xas[a]<=batas[a])
        {
          s[a]=true;
        }
        else{s[a]=false;}
      }
      else{
        if(xas[a]>=batas[a])
        {
          s[a]=true;
        }
        else{s[a]=false;}
      }
    }
  }
  
  if(s[0]==true && s[1]==true && s[2]==true && s[3]==true && s[4]==true && s[5]==true && s[6]==true && s[7]==true){
    flag1++;
    ROS_INFO("clear");
    yaa[0]=xaa[0];
    yaa[1]=xaa[1];
    yaa[2]=xaa[2];
  }
}

 
int main(int argc, char **argv)
{
  flag1=0;

  ros::init(argc, argv, "Move_Control");
  ros::NodeHandle n;
  ros::param::get("TELEOP_SPEED", speed);
  ros::param::get("TELEOP_SPEED", turn);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher head_pub_ = n.advertise<geometry_msgs::Twist>("/head_Tws", 1);
  ros::Publisher imu_override_pub_ = n.advertise<std_msgs::Bool>("/imu/imu_override", 100);
  ros::Publisher leg_height_pub_ = n.advertise<std_msgs::Bool>("/leg", 100);
  ros::Publisher state_pub_ = n.advertise<std_msgs::Bool>("/state", 100);
  ros::Publisher Led = n.advertise<std_msgs::Int32>("/led_control", 10);
  ros::Subscriber sub = n.subscribe("/euler_topic", 10, eulerCallback);
  ros::Subscriber tof_sub = n.subscribe("/tof_distances", 10, tofdistancesCallback);
  ros::Subscriber ultralytics_sub = n.subscribe("bounding_box", 10, boundingBoxCallback);

  ros::Subscriber _sub1 = n.subscribe("/chatter1", 1, chatter1Callback);
  ros::Subscriber _sub2 = n.subscribe("/chatter2", 1, chatter2Callback);
  ros::Subscriber _sub3 = n.subscribe("/chatter3", 1, chatter3Callback);

  ros::Rate r(100); 
  while (ros::ok())
  {
      // Execution

      avoidance();
      kontrol(a_gerak[flag1],flag1);
      
      state_pub_.publish(state_);
      pub.publish(twist);
      imu_override_pub_.publish(imu_override_);
      leg_height_pub_.publish(leg_height_);
      head_pub_.publish(head_Tws);
      Led.publish(Led_);

      ROS_INFO("step: %d command: %c", flag1, a_gerak[flag1]);


    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
