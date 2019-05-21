#include "myPoint.h"
#include "node_pid.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define TOLERANCE 0.2  // Distance from the target distance, at which the distance will be considered as achieved.
#define TOLERANCE_ANGLE 0.02  // Differenc from target angle, which will be tolerated.
#define MAX_SPEED 0.5    // Maximum speed of robot.
#define MAX_A_SPEED 0.314    // Maximum angular speed of robot.
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/global_odom"
#define SUBSCRIBER_TOPIC "odom"
#define PI 3.141592

double F_KP, F_KD, F_KI, R_KP, R_KD, R_KI, target_x, target_y, speed1, speed2, i_param, acc_time;

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "pid");
  ros::NodeHandle n;

//  n.param<double>("FKP",F_KP, 2.58);
//  n.param<double>("FKD",F_KD, 0.047);
//  n.param<double>("FKI",F_KI, 0);
//  n.param<double>("RKP",R_KP, 2);
//  n.param<double>("RKD",R_KD, 0.1);
//  n.param<double>("RKI",R_KI, 0);

  n.getParam("FKP", F_KP);
  n.getParam("FKD", F_KD);
  n.getParam("FKI", F_KI);
  n.getParam("RKP", R_KP);
  n.getParam("RKD", R_KD);
  n.getParam("RKI", R_KI);
  n.getParam("target_x",target_x);
  n.getParam("target_y",target_y);
  n.getParam("speed1",speed1);
  n.getParam("speed2",speed2);
  n.getParam("i_param",i_param);
  n.getParam("acc_time",acc_time);

  PID mypid;
  mypid.F_KP = F_KP;
  mypid.F_KD = F_KD;
  mypid.F_KI = F_KI;
  mypid.R_KP = R_KP;
  mypid.R_KD = R_KD;
  mypid.R_KI = R_KI;
  mypid.speed1 = speed1;
  mypid.speed2 = speed2;
  mypid.i_param = i_param;
  mypid.acc_time = acc_time;

  POINT temppoint;
  vector<POINT> traj;
  ifstream file("/home/nvidia/four_wheel_car/src/target_controller/config/point.txt");
  while(!file.eof()){
      file >> temppoint.x_pos;
      file >> temppoint.y_pos;
      traj.push_back(temppoint);
  }
  traj.pop_back();


//  POINT p1;
//  p1.x_pos = 4.0;
//  p1.y_pos = 1;
//
//  POINT p2;
//  p2.x_pos = 12;
//  p2.y_pos = 1;
//
//  traj.push_back(p1);
//  traj.push_back(p2);






//  float distance = 0, angle = 0.174;

  //Creating publisher
  ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
  ros::Publisher pubPidb = n.advertise<std_msgs::Float32>("/pid_param", PUBLISHER_BUFFER_SIZE);
  ros::Publisher pubPida = n.advertise<std_msgs::Float32>("/pid_paramang", PUBLISHER_BUFFER_SIZE);
  ros::Publisher pubPidt = n.advertise<std_msgs::Float32>("/pid_paratarget", PUBLISHER_BUFFER_SIZE);

  //Creating object, which stores data from sensors and has methods for
  //publishing and subscribing
  NodePID *nodePID = new NodePID(pubMessage, pubPidb, pubPida,pubPidt, TOLERANCE, TOLERANCE_ANGLE, MAX_SPEED, MAX_A_SPEED, mypid, traj);

  //Creating subscriber and publisher
  ros::Subscriber subscan = n.subscribe("/scan", SUBSCRIBER_BUFFER_SIZE, &NodePID::ScanCallback, nodePID);
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodePID::messageCallback, nodePID);
  ros::spin();

  return 0;
}
