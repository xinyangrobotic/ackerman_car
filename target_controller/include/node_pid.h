#ifndef NODEPID_H
#define NODEPID_H

#include "myPoint.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <queue>
#include <vector>
#include <fstream>

using namespace std;

typedef struct pid
{
    double F_KP;
    double F_KD;
    double F_KI;
    double R_KP;
    double R_KD;
    double R_KI;
    double speed1;
    double speed2;
    double i_param;
    double acc_time;
} PID;

typedef struct point{
    double x_pos;
    double y_pos;
    int turn;
} POINT;

class NodePID
{
public:

  /* Constructor:
   * 
   * pub     Publisher, which can send commands to robot.
   * tol     Position tolerance [link](#m).
   * tolA    Angle tolerance [link](#m).
   * dist    Robot will go forward this distance.
   * ang     Robot will turn by this angle [link](#rad).
   * mSpeed  Maximum speed of robot.
   * mASpeed Maximum angular speed of robot.
   */
  NodePID(ros::Publisher pubm, ros::Publisher pubp, ros::Publisher puba,ros::Publisher pubt, double tol, double tolA, double mSpeed, double mASpeed, PID mypid, vector<POINT> traj);

  ~NodePID();

  /* This method publishes commands for robot.
   *
   * angleCommand   Angular velocity.
   * speedCommand   Velocity.
   */
  void publishMessage(double angleCommand, double speedCommand);
  void publishScanMessage();
  void publishPid(double line_b, double msgang, double target);

  /* This method reads data from sensor and processes them to variables.
   * It saves actual position, calls methods for evaluating 
   * speed and calls method for publishing message.
   * 
   * msg   Message, which contains odometry data.
   */
  void messageCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /* This method calculates, if robot is close enough from target point.
   * If distance between robot and target point is shorter than 
   * tolerance, target is accomplished and method returns true.
   * 
   * actual   Actual position of robot.
   */
  bool closeEnough(MyPoint* actual);
  bool anglecloseEnough(MyPoint* actual);
  bool discloseEnough(MyPoint* actual);
  double line_b(MyPoint* actual);
  void line_param();
  void line_turn();



  /* This method calculates action intervention from PSD controller.
   * 
   * actual      Actual position of robot.
   * actualValue Actual output value.
   * lastValue   Output value from one step ago.
   * reference   Reference value.
   * kP          P constant for controller PSD.
   * kD          D constant for controller PSD.
   * kS          S constant for controller PSD.
   * sum         sum of errors.
   */
  double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum);

//variables
  MyPoint *start;     // Start position. Distance will be measured from here
  MyPoint *last;      // Last position of robot.
  MyPoint *_target;
  double tolerance;   // Tolerated deviation from target distance.
  double maxSpeed;    // Maximal velocity.
  double maxASpeed;   // Maximal angular velocity.
  ros::Publisher pubMessage; // Object for publishing messages.
  ros::Publisher pubPidb;
  ros::Publisher pubPidang;
  ros::Publisher pubPidtarget;
  double sumDistance;    // Sum of distance errors for PSD controller.
  double sumAngle;       // Sum of angle errors for PSD controller.
  double sumA;
  double sumB;
  double toleranceAngle; // Tolerated deviation from target angle.
  double _targetangle;
  double _targetdistance;
  PID _mypid;
  int stage;
  int stage_turn;
  bool initial;
  double _a, _b, _c;
  double _a_current, _b_current, _c_current, _a_forward, _b_forward, _c_forward;
  double angleMinLeft;       // Angle, at which was measured the shortest distance on the left.
  double distMinLeft;        // Minimum distance masured by sensor on the left.
  double angleMinRight;      // Angle, at which was measured the shortest distance on the right.
  double distMinRight;
  bool stop;
  double angleCommand;
  double speedCommand;
  double pidspeed;
  double acc_time;
  double angle_error;
  double _time_temp;
  double _speed_temp;
  vector<POINT> _traj;
  int _reach;
  int _totalgoal;
  double _turn_r;
  double _turn_delta;
};

#endif

