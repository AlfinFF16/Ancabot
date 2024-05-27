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
#include <cmath>

float ping[7]={0,0,0,0,0,0,0};
// front tof, back tof, left tof, right tof, imu roll, mu yaw, center pose of detected object

void tofdistancesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  for (int i=0;i<4;i++){
    ping[i]=msg->data[i];
  }
  // ROS_INFO("ToF Reading: front=%f back=%f left=%f  right=%f", ping[0], ping[1], ping[2], ping[3]);
}

void eulerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Extract Euler angles from the received message
  ping[4] = msg->pose.position.x;     // roll orientation
  ping[5] = msg->pose.position.z;     // yaw orientation

  // ROS_INFO("IMU Orientation: roll=%f pitch=%f yaw=%f", ping[4], ping[5], ping[6]);
}

void boundingBoxCallback(const ultralytics_ros::BoundingBox::ConstPtr& msg)
{
    float x_center = 0;

    if (msg->class_name == "tensor(1.)" && msg->confidence >= 0.7)
    {
        x_center = msg->center_x;
    }
    else
    {
        x_center = 0;
    }

    ping[6] = x_center;

    // ROS_INFO("Center Coordinate of korban: x = %f", x_center);
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
char a_gerak[]  ={'d','d','w','D','a','w','s','s','x','D','D','x','d','w','d','w','s','x','a','D','w','a','w','s','s','x','d','w','Q','s','x','a','D','w','A','w','d','s','w','w','s','s','x','a','w','d','w','A','a','A','s','x','A','a','s'};
// step            0   1   2   3   4   5   6   7   8   9  10   11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41 42  43  44  45  46 

// Pengondisian step dan batas gerakan
std::map<int, std::vector<float>> step{
  // {step, {Tof_depan, Tof_belakang, Tof_kiri, Tof_kanan, Imu Roll, Imu Yaw, X Coord of Detected Object, Gripper (lifter), Gripper (gripper), Speed, Turn}}
  // gripper: teleop 'o' --> {0,0}; teleop 'p' --> {-2,0}; teleop 'l' --> {0,-1}; teleop ';' --> {-1,-1}
  {0,   {0,0,0,0,         -200,-5,      0,    -2,0,1,1}},   // keluar dari home (handling pengondisian untuk orientasi berbeda)
  {1,   {540,0,0,100,     -200,-90,     0,    -2,0,1,1}},   // keluar dari home
  {2,   {0,520,100,0,     -200,-200,    0,    -2,0,1,1}},   // menuju zona K1
  {3,   {0,0,0,70,        -200,-200,    0,    -2,0,1,1}},   // menyamping bersiap untuk ke K1
  {4,   {280,220,350,0,   -200,-200,  160,    -2,0,1,1}},   // berotasi hingga gripper sejajar K1
  {5,   {180,190,0,0,     -200,-200,    0,    0,-1,1,1}},   // mendekati K1 (gripper diturunkan dan terbuka) 
  {6,   {140,0,0,0,       -200,-200,    0,     0,0,1,1}},   // di posisi K1 dan gripper men-grip korban
  {7,   {140,0,0,0,       -200,-200,    0,    -2,0,1,1}},   // gripper dengan korban diangkat kembali
  {8,   {0,80,0,0,        -200,-200,    0,    -2,0,1,1}},   // keluar dari zona K1
  {9,   {0,0,0,0,           10,-200,    0,    -2,0,1,1}},   // bergerak menyamping melewati Jalan Retak  
  {10,  {500,0,0,190,       10,-200,    0,    -2,0,1,1}},   // bergerak menyamping melewati Turunan
  {11,  {0,60,0,0,        -200,-200,    0,    -2,0,2,1}},   // mundur untuk pemosisisan
  {12,  {300,100,0,120,   -200,-90,     0,    -2,0,2,1}},   // berotasi di jalan berbatu
  {13,  {180,0,500,0,     -200,-200,    0,    -2,0,2,1}},   // bergerak maju melewati Bebatuan - Menuju SZ1
  {14,  {200,220,400,0,   -200,-115,    0,    -2,0,1,1}},   // berotasi hingga sejajar dengan SZ1
  {15,  {220,500,0,0,     -200,-200,    0,    -2,0,1,1}},   // mendekat pada SZ1
  {16,  {100,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // menurunkan K1 di SZ1
  {17,  {200,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // mundur dengan gripper masih terbuka (handling korban terangkat lagi)
  {18,  {600,200,300,0,   -200,-200,    0,    -2,0,1,1}},   // berotasi hingga body sejajar rintangan kelereng
  {19,  {0,0,0,90,        -200,-200,    0,    -2,0,1,1}},   // menyamping ke kanan
  {20,  {480,480,280,0,   -200,-200,    0,    -2,0,1,1}},   // maju hingga tegak lurus K2
  {21,  {300,200,0,300,   -200,-200,  160,    -2,0,1,1}},   // berotasi hingga sejajar K2
  {22,  {150,270,0,0,     -200,-200,    0,    0,-1,1,1}},   // maju mendekat K2
  {23,  {140,0,0,0,       -200,-200,    0,     0,0,1,1}},   // di posisi K2 dan gripper men-grip korban
  {24,  {140,0,0,0,       -200,-200,    0,    -2,0,1,1}},   // gripper dengan korban diangkat kembali
  {25,  {0,150,0,0,       -200,-200,    0,    -2,0,1,1}},   // mundur dari zona pengambilan K2
  {26,  {200,300,0,100,   -200,-35,     0,    -2,0,1,1}},   // berotasi menyesuaikan ke SZ2
  {27,  {240,400,0,270,   -200,-200,    0,    -2,0,1,1}},   // menyamping menyesuaikan ke SZ2
  {28,  {220,0,0,0,       -200,-200,    0,    -2,0,1,1}},   // maju menyesuaikan ke SZ2
  {29,  {100,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // menurunkan K2 di SZ2
  {30,  {260,0,200,200,   -200,-200,    0,    0,-1,1,1}},   // mundur dari SZ2
  {31,  {0,320,0,180,     -200,90,      0,    -2,0,1,1}},   // berotasi untuk keluar dari area SZ2
  {32,  {0,0,0,90,        -200,-200,    0,    -2,0,1,1}},   // menyamping ke kanan mendekati dinding
  {33,  {100,500,280,0,   -200,-200,    0,    -2,0,1,1}},   // maju keluar dari area SZ2
  {34,  {500,140,100,0,   -200,-200,    0,    -2,0,1,1}},   // menyamping bersiap menuju jalan retak
  {35,  {600,0,0,90,      -200,-200,    0,    -2,0,1,1}},   // maju menuju K4
  {36,  {250,150,300,300, -200,-200,  160,    -2,0,1,1}},   // berotasi hingga sejajar dengan K4
  {37,  {170,0,0,0,       -200,-200,    0,     0,0,1,1}},   // menurunkan gripper
  {38,  {0,300,0,0,       -200,-200,    0,     0,0,1,1}},   // maju menuju K4 dengan gripper tertutup
  {39,  {0,320,0,0,       -200,-200,    0,    0,-1,1,1}},   // maju menuju K4 dengan gripper terbuka
  {40,  {140,0,0,0,       -200,-200,    0,     0,0,1,1}},   // di posisi K4 dan gripper men-grip korban
  {41,  {140,0,0,0,       -200,-200,    0,    -2,0,1,1}},   // gripper dengan korban diangkat kembali
  {42,  {300,170,0,0,     -200,-200,    0,    -2,0,1,1}},   // mudur dari zona K4
  {43,  {500,500,100,0,   -200,90,      0,    -2,0,1,1}},   // berotasi untuk keluar dari zona K4 dan rintangan jalan retak
  {44,  {250,0,0,240,     -200,-200,    0,    -2,0,1,1}},   // maju bersiap menuju tangga
  {45,  {300,200,0,0,     -200,-200,    0,    -2,0,1,1}},   // berotasi berisiap menuju tangga
  {46,  {120,350,0,0,     -200,-200,    0,    -2,0,1,1}},   // maju bersiap menuju tangga
  {47,  {0,600,230,0,      -10,-200,    0,    -2,0,1,1}},   // menaiki tangga
  {48,  {300,0,500,80,    -200,90,      0,    -2,0,1,1}},   // berotasi hingga sejajar SZ4
  {49,  {0,0,0,140,       -200,-200,    0,    -2,0,1,1}},   // menyamping higga colinear SZ4
  {50,  {100,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // menurunkan K4 di SZ4
  {51,  {320,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // mundur bersiap melewati rintangan longsor
  {52,  {0,200,120,0,     -200,-200,    0,    -2,0,1,1}},   // menyamping hingga berada di zona finish
  {53,  {110,0,470,100,   -200,-200,    0,    -2,0,1,1}},   // berotasi di zona finish
  {54,  {0,0,0,0,0,       -200,-200,    0,    0,-1,1,1}},   // finish
};


std::map<int, std::vector<bool>> _f_{
  // komparator (0)(sensor>=batas) (1)(Sensor<=batas) (index 0 - 6)
  // Leg Height - tall = (1) && short = (0) (index 9) 
  {0,   {0,0,0,0,0,1,0, 1,0,0}},
  {1,   {0,0,0,1,0,1,0, 1,0,0}},
  {2,   {0,0,0,0,0,0,0, 1,0,0}},
  {3,   {0,0,0,1,0,0,0, 1,0,0}},
  {4,   {1,1,0,0,0,0,0, 1,0,0}}, 
  {5,   {1,0,0,0,0,0,0, 1,0,0}},
  {6,   {1,0,0,0,0,0,0, 1,0,0}},
  {7,   {0,0,0,0,0,0,0, 1,0,0}},
  {8,   {0,1,0,0,0,0,0, 1,0,0}},
  {9,   {0,0,0,0,0,0,0, 1,0,1}},
  {10,  {0,0,0,1,1,0,0, 1,0,1}},
  {11,  {0,1,0,0,0,0,0, 1,0,1}},
  {12,  {1,0,0,1,0,1,0, 1,0,1}},
  {13,  {1,0,0,0,0,0,0, 1,0,1}},
  {14,  {0,0,1,0,0,1,0, 1,0,1}},
  {15,  {1,0,0,0,0,0,0, 1,0,1}},
  {16,  {1,0,0,0,0,0,0, 1,0,1}},
  {17,  {1,0,0,0,0,0,0, 1,0,1}},
  {18,  {0,1,0,0,0,0,0, 1,0,1}},
  {19,  {0,0,0,1,0,0,0, 1,0,1}},
  {20,  {1,0,1,0,0,0,0, 1,0,1}},
  {21,  {1,1,0,0,0,0,0, 1,0,1}},
  {22,  {1,0,0,0,0,0,0, 1,0,1}},
  {23,  {1,0,0,0,0,0,0, 1,0,1}},
  {24,  {0,0,0,0,0,0,0, 1,0,1}},
  {25,  {0,1,0,0,0,0,0, 1,0,1}},
  {26,  {0,0,0,1,0,1,0, 1,0,1}},
  {27,  {0,1,0,0,0,0,0, 1,0,1}},
  {28,  {1,0,0,0,0,0,0, 1,0,1}},
  {29,  {1,0,0,0,0,0,0, 1,0,1}},
  {30,  {0,0,0,0,0,0,0, 1,0,1}},
  {31,  {0,1,0,1,0,0,0, 1,0,1}},
  {32,  {0,0,0,1,0,0,0, 1,0,1}},
  {33,  {0,1,0,0,0,0,0, 1,0,1}},
  {34,  {0,1,1,0,0,0,0, 1,0,1}},
  {35,  {1,0,0,1,0,0,0, 1,0,1}},
  {36,  {1,1,0,0,0,0,1, 1,0,1}},
  {37,  {1,0,0,0,0,0,0, 1,0,1}},
  {38,  {0,0,0,0,0,0,0, 1,0,1}},
  {39,  {0,0,0,0,0,0,0, 1,0,1}},
  {40,  {1,0,0,0,0,0,0, 1,0,1}},
  {41,  {0,0,0,0,0,0,0, 1,0,1}},
  {42,  {0,1,0,0,0,0,0, 1,0,1}},
  {43,  {0,0,1,0,0,0,0, 1,0,1}},
  {44,  {1,0,0,0,0,0,0, 1,0,1}},
  {45,  {0,1,0,0,0,1,0, 1,0,1}},
  {46,  {0,1,0,0,0,0,0, 1,0,1}},
  {47,  {0,0,1,0,0,0,0, 1,0,1}},
  {48,  {0,0,0,0,0,0,0, 1,0,1}},
  {49,  {0,0,0,0,0,0,0, 1,0,1}},
  {50,  {1,0,0,0,0,0,0, 1,0,1}},
  {51,  {0,0,0,0,0,0,0, 1,0,1}},
  {52,  {0,1,1,0,0,0,0, 1,0,1}},
  {53,  {1,0,0,1,0,0,0, 1,0,1}},
  {54,  {1,1,1,1,1,1,1, 1,0,1}},
};

// Init variables
float speed(1);                                                     // Linear velocity (m/s)
float turn(1);                                                      // Angular velocity (rad/s)
float x(0), y(0), z(0), xa(0), ya(0), za(0), xb(0), yb(0), th(0);   // Forward/backward/neutral direction vars
char key(' ');
int offset(5);
bool isAvoidanceActive = false;
int currentStep = 0;
geometry_msgs::Twist twist;
geometry_msgs::Twist head_Tws;
std_msgs::Bool imu_override_;
std_msgs::Bool leg_height_;
std_msgs::Bool state_;

void avoidance(){
  if (ping[0] != 0 && ping[1] !=0 && ping[2] != 0 && ping[3] != 0)
  {
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
        twist.linear.x = -0.2;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
      }
        if(ping[1] <= 40){
        //gerakan maju
        twist.linear.x = 0.5;
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
    }
    else 
    {
      isAvoidanceActive = false;
    }
  }
}

bool pilih;
bool diffOrient;
void kontrol(char arah_, int step_){
  
  if (isAvoidanceActive) {
    return;
  }

  if (step_ == 0)
  {
    // Conditioning for robot first orientation
    if(ping[3] > ping[2])         // if the right's measurement greater than left's
    {
      key = arah_;
      diffOrient = false;
    }
    else if(ping[3] < ping[2])    // if the right's measurement smaller than left's
    {
      key = 'a';
      diffOrient = true;
    }
  }
  else if (step_ == 1 && diffOrient)
  {
    key = 'a';
  }
  else
  {
    key = arah_;
  }

  int batas[7];
  if (step.count(step_) == 1)
    {
      for(int a=0;a<7;a++){
        batas[a]=step[step_][a];
        if (diffOrient)
        {
          switch(step_)
          {
            case 0: batas[5] = 5; break;
            case 1: batas[5] = 90; break;
            case 12: batas[5] = 90; break;
            case 14: batas[5] = 45; break;
            case 18: batas[5] = -180; break;
            case 26: batas[5] = 145; break;
            case 31: batas[5] = -90; break;
            case 43: batas[5] = -90; break;
            case 49: batas[5] = -90; break;
            default: break;
          }
        }
      }
      // currentStep = step_;
      xb=step[step_][7];
      yb=step[step_][8];
      speed=step[step_][9];
      turn=step[step_][10];
    }

  bool flag_[7];
  if (_f_.count(step_) == 1)
    {
      for(int a=0;a<7;a++){
        flag_[a]=_f_[step_][a];
        if (diffOrient)
        {
          switch(step_)
          {
            case 0: flag_[5] = 0; break;
            case 1: flag_[5] = 0; break;
            case 12: flag_[5] = 1; break;
            case 14: flag_[5] = 1; break;
            case 18: flag_[5] = 1; break;
            case 26: flag_[5] = 1; break;
            case 31: flag_[5] = 0; break;
            case 43: flag_[5] = 0; break;
            case 48: flag_[5] = 0; break;
            default: break;
          }
        }
      }
    pilih=_f_[step_][7];
    imu_override_.data = _f_[step_][8];
    leg_height_.data = _f_[step_][9];
    }

  // Trying to stabilize the orientation of robot's body
  if (moveBindings.count(key) == 1 && (step_ == 13 || step_ == 33 || step_ == 35))
  {
    float desired_yaw = 90.0;
    float yaw_difference = desired_yaw - std::abs(ping[6]);
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
           
      // ROS_INFO("\rCurrent: speed %f   | turn %f | Last command: %c   ", speed, turn, key);
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
           
      // ROS_INFO("\rCurrent: speed %f   | turn %f | Last command: %c   ", speed, turn, key);
    }
  }

    state_.data = true;

    // Update the Twist message
    twist.linear.x = x * speed * 0.5;
    twist.linear.y = y * speed * 0.5;
    twist.linear.z = z * speed * 0.5;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn * 0.5;

    head_Tws.linear.x = xb * 0.5  ; //lifter
    head_Tws.linear.y = yb * 1.01 ; //gripper

    ROS_INFO("Current Step: %d | Command: %c | Speed: %.1f| Turn: %.1f", step_, key, speed, turn);
    ROS_INFO("Limits: %d, %d, %d, %d, %d, %d, %d", batas[0], batas[1], batas[2], batas[3], batas[4], batas[5], batas[6]);
    ROS_INFO("Measurements: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",ping[0],ping[1],ping[2],ping[3],ping[4],ping[5],ping[6]);
    ROS_INFO("Flags: %d, %d, %d, %d, %d, %d, %d",flag_[0],flag_[1],flag_[2],flag_[3],flag_[4],flag_[5],flag_[6]);


    bool s[7]={false,false,false,false,false,false};

  if(pilih==true){
    for (int a=0; a<7; a++){
    for (int a = 0; a < 7; a++) {
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
    for (int a=0; a < 7; a++){
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
  
  if(s[0]==true && s[1]==true && s[2]==true && s[3]==true && s[4]==true && s[5]==true && s[6]==true){
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
  ros::Subscriber euler_sub = n.subscribe("/euler_topic", 10, eulerCallback);
  ros::Subscriber tof_sub = n.subscribe("/tof_distances", 10, tofdistancesCallback);
  ros::Subscriber ultralytics_sub = n.subscribe("/bounding_box", 10, boundingBoxCallback);

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

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
