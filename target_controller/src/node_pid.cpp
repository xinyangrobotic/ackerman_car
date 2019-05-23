#include "node_pid.h"
#include <math.h>
#define PI 3.141592
//#define F_KP 2.58  // P constant for PSD translation controller
//#define F_KD 0.047  // D constant for PSD translation controller
//#define F_KI 0.0  // S constant for PSD translation controller
//#define R_KP 2.0  // P constant for PSD rotation controller
//#define R_KD 0.1  // D constant for PSD rotation controller
//#define R_KI 0.0  // S constant for PSD rotation controller

NodePID::NodePID(ros::Publisher pubm, ros::Publisher pubp, ros::Publisher puba, ros::Publisher pubt, double tol, double tolA, double mSpeed, double mASpeed, PID mypid, vector<POINT> traj)
{
  stage = 0;
  stage_turn = 0;
  tolerance = tol;
  toleranceAngle = tolA;
  maxSpeed = mypid.speed2;
  maxASpeed = mASpeed;  
  pubMessage = pubm;
  pubPidb = pubp;
  pubPidang = puba;
  pubPidtarget = pubt;
  sumDistance = 0;
  sumAngle = 0;
  start = new MyPoint(0.0, 0.0, 0.0, 0, ros::Time::now());
  last = new MyPoint(0.0, 0.0, 0.0, 0, ros::Time::now());
  _target = new MyPoint(0.0, 0.0, 0.0, 0, ros::Time::now());
  _mypid = mypid;
  stop = false;
  angleCommand = 0;
  speedCommand = 0;
  sumA = 0;
  sumB = 0;
  acc_time = _mypid.acc_time;
  initial = true;
  angle_error = 0;
  _time_temp = ros::Time::now().toSec();
  _speed_temp = 0;
  _traj = traj;
  _reach = 0;
  _totalgoal = _traj.size();
  _turn_r = 0;
  _turn_delta = 0;

}

NodePID::~NodePID()
{
}

void NodePID::line_param(){
    _a = start->y - _target->y;
    _b = _target->x - start->x;
    _c = start->x * _target->y - _target->x * start->y;
}

void NodePID::line_turn(){
    _a_current = _traj[_reach-2].y_pos - _traj[_reach-1].y_pos;
    _b_current = _traj[_reach-1].x_pos - _traj[_reach-2].x_pos;
    _c_current = _traj[_reach-2].x_pos * _traj[_reach-1].y_pos - _traj[_reach-1].x_pos * _traj[_reach-2].y_pos;

    _a_forward = _b_current;
    _b_forward = -_a_current;
    _c_forward = _a_current * _traj[_reach].y_pos - _b_current * _traj[_reach].x_pos;
    ROS_INFO_STREAM("a_c"<<_a_current<<"b_c"<<_b_current<<"c_c"<<_c_current<<" "<<"a_f"<<_a_forward<<"b_f"<<_b_forward<<"c_f"<<_c_forward);
}

double NodePID::line_b(MyPoint* actual){
    double b = fabs(_a * actual->x + _b * actual->y + _c)/sqrtf(pow(_a, 2) + pow(_b, 2));
    return b;
}

//Publisher
void NodePID::publishMessage(double angleCommand, double speedCommand)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = speedCommand;
  msg.angular.z = angleCommand;
  ROS_INFO_STREAM("angleCommand:"<<angleCommand);

  //publishing message
  pubMessage.publish(msg);
}

//Publisher
void NodePID::publishPid(double line_b, double msgangle, double target)
{
    //preparing message
    std_msgs::Float32 msg_b;
    std_msgs::Float32 msg_ang;
    std_msgs::Float32 msg_target;
    if(msgangle > 0.314){
        msgangle = 0.314;
    } else if(msgangle < -0.314){
        msgangle = -0.314;
    }
    msg_b.data = line_b;
    msg_ang.data = msgangle;
    msg_target.data = target;
//    ROS_INFO_STREAM("line_msg:"<<line_b);
    //publishing message

    pubPidb.publish(msg_b);
    pubPidang.publish(msg_ang);
    pubPidtarget.publish(msg_target);
}

void NodePID::publishScanMessage(){

    if(distMinLeft < 1 && distMinRight >1){
        ROS_INFO_STREAM("Boom! You have a trouble on your left, please stop now!");
        stop = true;
    } else if(distMinLeft > 1 && distMinRight < 1){
        ROS_INFO_STREAM("Boom! You have a trouble on your right, please stop now!");
        stop = true;
    }else if(distMinLeft < 1 && distMinRight <1){
        ROS_INFO_STREAM("Boom! You have a trouble, please stop now!");
        stop = true;
    } else{
        stop = false;
    }


}

//Subsciber
void NodePID::ScanCallback(const sensor_msgs::LaserScanConstPtr& msg){
    //Calculation of array size from angle range
    int size = msg->ranges.size();
    int minIndexLeft = 0;
    int minIndexRight = size/2;

    // goes throgh  array and find minimum  on the right

    for(int i=0;i<size/2;i++){
        if(msg->ranges[i] < msg->ranges[minIndexRight] && msg->ranges[i] > 0.05)
            minIndexRight = i;
    }

// goes throgh  array and find minimum  on the left

    for(int i=size/2;i<size;i++){
        if(msg->ranges[i] < msg->ranges[minIndexLeft] && msg->ranges[i] > 0.05)
            minIndexLeft = i;
    }

    //Calculaion of angle
    angleMinLeft = (minIndexLeft-size/2)*msg->angle_increment;
    distMinLeft = msg->ranges[minIndexLeft];
    angleMinRight = (minIndexRight-size/2)*msg->angle_increment;
    distMinRight = msg->ranges[minIndexRight];
//    publishScanMessage();
}

//Subscriber
void NodePID::messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double angleCommand = 0;
  double speedCommand = 0;
  double speed = 0;

    _target->x = _traj[_reach].x_pos;
    _target->y = _traj[_reach].y_pos;
    _target->turn = _traj[_reach].turn;

  MyPoint* actual = new MyPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation), 0, msg->header.stamp);

  if (discloseEnough(actual) == true)
    {
        if(_reach == _totalgoal - 1){
            ROS_INFO_STREAM("Goal ACHIEVED!");
            publishMessage(0.0, 0.0);
            exit(0);
        } else {
            ROS_INFO_STREAM("Point: " << _reach << " ACHIEVED!");
            _reach = _reach + 1;
            start->x = actual->x;
            start->y = actual->y;
            start->time = actual->time;
            start->angle = actual->angle;
            _target->x = _traj[_reach].x_pos;
            _target->y = _traj[_reach].y_pos;
            _target->turn = _traj[_reach].turn;

        }
    }




  if (initial == true)
  {
    start->x = actual->x;
    start->y = actual->y;
    start->time = actual->time;
    start->angle = actual->angle;
    start->turn = actual->turn;
    last->x = actual->x;
    last->y = actual->y;
    last->time = actual->time;
    last->angle = actual->angle;
    last->turn = actual->turn;

    stage = 3;
    _time_temp = ros::Time::now().toSec();
    _speed_temp = fmin(maxSpeed,speedCommand);
    initial = false;
  }

  if(stop == false) {
      ROS_INFO_STREAM("start_x: "<<start->x<<"start_y: "<<start->y);

      /**等于0即说明此时到目标点不需要转弯**/
      ROS_INFO_STREAM("target_x: "<<_target->x<<"target_y: "<<_target->y<<"turn? "<<_target->turn);
      if(_target->turn == 0){
          ROS_INFO_STREAM("stage: "<< stage<<"point: "<<_reach);

          _targetangle = atan2(_target->y - start->y, _target->x - start->x);
          line_param();

          if (actual->angle - last->angle < -PI) {
              actual->angle += 2 * PI;
          } else if (actual->angle - last->angle > PI) {
              actual->angle -= 2 * PI;
          }

          double dis = line_b(actual);

          if (stage == 1 && fabs(dis) < 0.2) {
              stage = 3;
              sumA = 0;
              _time_temp = ros::Time::now().toSec();
              _speed_temp = fmin(maxSpeed,speedCommand);
          } else if ( stage ==3 && fabs(dis) < 0.2 && angle_error < 3) {
              stage = 2;
              sumB = 0;
              _time_temp = ros::Time::now().toSec();
              ROS_INFO_STREAM("STAGE_ANGLE_ERROR:"<<angle_error);
              _speed_temp = fmin(maxSpeed,speedCommand);
          }else if( stage == 3 && fabs(dis) > 0.2){
              stage = 1;
              sumA = 0;
              _time_temp = ros::Time::now().toSec();
              _speed_temp = fmin(maxSpeed,speedCommand);
          }else if(stage == 2 && fabs(dis) > 0.2){
              stage = 1;
              sumA = 0;
              _time_temp = ros::Time::now().toSec();
              _speed_temp = fmin(maxSpeed,speedCommand);
          }


          if(stage == 1) {
              /*看看当前的点在目标线的左侧还是右侧*/
              double y_fake = -(_c + _a * actual->x) / _b;
              int sign;
              if (y_fake < actual->y)
                  sign = 1;/*左侧*/
              else
                  sign = -1;/*右侧*/

              double actual_c = actual->angle;
              double actual_l = last->angle;
              double delta = 0.314;


              angleCommand = calculatePSD(actual, actual_c, actual_l, _targetangle + delta * -sign, _mypid.R_KP, _mypid.R_KD,
                                          _mypid.R_KI, &sumA);
              ROS_INFO_STREAM("line_b:" << line_b(actual) * sign<<"actual_c:"<<actual_c* 180/PI<<"target_ang: "<<(_targetangle + delta * -sign) * 180/PI);

              publishPid(actual_c, angleCommand, _targetangle + delta * -sign);


              double now = ros::Time::now().toSec();
              ROS_INFO_STREAM("now_time:" << now << "last" << _time_temp << "time_diff:" << (now - _time_temp));
              if ((now - _time_temp) >= acc_time) {
                  speed = _mypid.speed1;
              } else {
                  speed = (now - _time_temp) / acc_time * (_mypid.speed1 - _speed_temp) + _speed_temp;
              }

              //新加的角度判别模块
              if (speedCommand == 0 && angleCommand != 0) {
                  speedCommand = speed;
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              } else if (angleCommand == 0 && speedCommand == 0) {
                  speedCommand = speed;
              } else
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              //Saving position to last
              last->x = actual->x;
              last->y = actual->y;
              last->time = actual->time;
              last->angle = actual->angle;
              angle_error = fabs(tf::getYaw(msg->pose.pose.orientation) * 180 / PI - _targetangle * 180 / PI);
              ROS_INFO_STREAM("angle_error:" << angle_error);

          }

          if (stage == 3) {

              /*看看当前的点在目标线的左侧还是右侧*/
              double y_fake = -(_c + _a * actual->x) / _b;
              int sign;
              if (y_fake < actual->y)
                  sign = 1;/*左侧*/
              else
                  sign = -1;/*右侧*/
              double actual_c = actual->angle;
              double actual_l = last->angle;

              angleCommand = calculatePSD(actual, actual_c, actual_l, _targetangle, _mypid.R_KP, _mypid.R_KD,
                                          _mypid.R_KI, &sumA);
              ROS_INFO_STREAM("line_b:" << line_b(actual) * sign<<"actual_c:"<<actual_c* 180/PI<<"target_ang: "<<(_targetangle) * 180/PI);
              publishPid(actual_c, angleCommand, _targetangle);

              double now = ros::Time::now().toSec();
              ROS_INFO_STREAM("now_time:" << now << "last" << _time_temp << "time_diff:" << (now - _time_temp));
              if ((now - _time_temp) >= acc_time) {
                  speed = _mypid.speed1;
              } else {
                  speed = (now - _time_temp) / acc_time * (_mypid.speed1 - _speed_temp) + _speed_temp;
              }

              //新加的角度判别模块
              if (speedCommand == 0 && angleCommand != 0) {
                  speedCommand = speed;
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              } else if (angleCommand == 0 && speedCommand == 0) {
                  speedCommand = speed;
              } else
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              //Saving position to last
              last->x = actual->x;
              last->y = actual->y;
              last->time = actual->time;
              last->angle = actual->angle;
              angle_error = fabs(tf::getYaw(msg->pose.pose.orientation) * 180 / PI - _targetangle * 180 / PI);
              ROS_INFO_STREAM("angle_error:" << angle_error<<"speed:"<<speed);

          }


          if (stage == 2) {

              /*看看当前的点在目标线的左侧还是右侧*/
              double y_fake = -(_c + _a * actual->x) / _b;
              int sign;
              if (y_fake < actual->y)
                  sign = 1;/*左侧*/
              else
                  sign = -1;/*右侧*/
              double actual_c = actual->angle;
              double actual_l = last->angle;

              angleCommand = calculatePSD(actual, actual_c, actual_l, _targetangle, _mypid.F_KP, _mypid.F_KD,
                                          _mypid.F_KI, &sumB);
              ROS_INFO_STREAM("line_b:" << line_b(actual) * sign<<"actual_c:"<<actual_c* 180/PI<<"target_ang: "<<(_targetangle) * 180/PI);
              publishPid(actual_c, angleCommand, _targetangle);


              double now = ros::Time::now().toSec();
              ROS_INFO_STREAM("now_time:" << now << "last" << _time_temp << "time_diff:" << (now - _time_temp));
              if ((now - _time_temp) >= acc_time) {
                  speed = _mypid.speed2;
              } else {
                  speed = (now - _time_temp) / acc_time * (_mypid.speed2 - _speed_temp) + _speed_temp;
              }

              //新加的角度判别模块
              if (speedCommand == 0 && angleCommand != 0) {
                  speedCommand = speed;
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              } else if (angleCommand == 0 && speedCommand == 0) {
                  speedCommand = speed;
              } else
                  angleCommand = atan(angleCommand * 0.55 / speedCommand);
              //Saving position to last
              last->x = actual->x;
              last->y = actual->y;
              last->time = actual->time;
              last->angle = actual->angle;
              angle_error = fabs(tf::getYaw(msg->pose.pose.orientation) * 180 / PI - _targetangle * 180 / PI);
              ROS_INFO_STREAM("angle_error:" << angle_error);

          }
      } else//此时需要通过转弯去目标点
          {

              if (actual->angle - last->angle < -PI) {
                  actual->angle += 2 * PI;
              } else if (actual->angle - last->angle > PI) {
                  actual->angle -= 2 * PI;
              }

              ROS_INFO_STREAM("stage_turn: "<< stage_turn<<"point: "<<_reach<<"actual_ang"<<actual->angle * 180/PI);
              line_turn();
              double d_n = fabs(_a_current * _traj[_reach].x_pos + _b_current * _traj[_reach].y_pos + _c_current)/sqrtf(pow(_a_current, 2) + pow(_b_current, 2));
              double d_f = fabs(_a_forward * actual->x + _b_forward * actual->y + _c_forward)/sqrtf(pow(_a_forward, 2) + pow(_b_forward, 2));
              _targetangle = atan2(_traj[_reach].y_pos - _traj[_reach-1].y_pos, _traj[_reach].x_pos - _traj[_reach-1].x_pos);
              ROS_INFO_STREAM("d_f:" << d_f<<"  "<<"d_n:" << d_n);
              double y_fake = -(_c_current + _a_current * _traj[_reach].x_pos) / _b_current;
              int sign;
              if (y_fake < _traj[_reach].y_pos)
                  sign = 1;/*左侧*/
              else
                  sign = -1;/*右侧*/

              _targetangle = atan2(_traj[_reach-1].y_pos - _traj[_reach-2].y_pos, _traj[_reach-1].x_pos - _traj[_reach-2].x_pos) + PI / 2 * (sign);

              if(d_n < d_f &&  stage_turn == 0){
                  stage_turn = 1;
                  ROS_INFO_STREAM("here0");
              } else if(d_n < d_f && stage_turn == 1){
                  stage_turn = 1;
                  ROS_INFO_STREAM("here1");
              }else if(d_n >= d_f && stage_turn == 0){
                  stage_turn = 2;
                  _turn_r = d_f;
                  ROS_INFO_STREAM("here2");
              } else if(d_n >= d_f && stage_turn == 1){
                  stage_turn = 2;
                  _turn_r = d_f;
                  ROS_INFO_STREAM("here3");
              }else if(d_f <= 0.2 && stage_turn == 2){
                  stage_turn = 3;
                  sumA = 0;
              } else if(d_f > 0.2 && stage_turn == 3){
                  stage_turn = 4;
                  sumA = 0;
              }else if(d_f <= 0.2 && stage_turn == 4){
                  stage_turn = 3;
                  sumA = 0;
              }

              if(stage_turn == 1){
                  speedCommand = _mypid.speed1;
              }

              if(stage_turn == 2){
                      angleCommand = atan(0.55 / (_turn_r)) * sign;
                      speedCommand = _mypid.speed1;
              }

              if(stage_turn == 3){

                  double actual_c = actual->angle;
                  double actual_l = last->angle;

                  angleCommand = calculatePSD(actual, actual_c, actual_l, _targetangle, _mypid.R_KP, _mypid.R_KD,
                                              _mypid.R_KI, &sumA);
                  ROS_INFO_STREAM("line_b:" << d_f<<"actual_c:"<<actual_c* 180/PI<<"target_ang: "<<(_targetangle) * 180/PI);
                  publishPid(actual_c, angleCommand, _targetangle);

                  speed = _mypid.speed1;

                  //新加的角度判别模块
                  if (speedCommand == 0 && angleCommand != 0) {
                      speedCommand = speed;
                      angleCommand = atan(angleCommand * 0.55 / speedCommand);
                  } else if (angleCommand == 0 && speedCommand == 0) {
                      speedCommand = speed;
                  } else
                      angleCommand = atan(angleCommand * 0.55 / speedCommand);

              }

              if(stage_turn == 4){


                      double actual_c = actual->angle;
                      double actual_l = last->angle;
                      double delta = 0.314;


                      angleCommand = calculatePSD(actual, actual_c, actual_l, _targetangle + delta * sign, _mypid.R_KP, _mypid.R_KD,
                                                  _mypid.R_KI, &sumA);
                      ROS_INFO_STREAM("line_b:" << d_f<<"actual_c:"<<actual_c* 180/PI<<"target_ang: "<<(_targetangle + delta * -sign) * 180/PI);

                      publishPid(actual_c, angleCommand, _targetangle + delta * sign);

                      speed = _mypid.speed1;

                      //新加的角度判别模块
                      if (speedCommand == 0 && angleCommand != 0) {
                          speedCommand = speed;
                          angleCommand = atan(angleCommand * 0.55 / speedCommand);
                      } else if (angleCommand == 0 && speedCommand == 0) {
                          speedCommand = speed;
                      } else
                          angleCommand = atan(angleCommand * 0.55 / speedCommand);
              }
              //Saving position to last
              last->x = actual->x;
              last->y = actual->y;
              last->time = actual->time;
              last->angle = actual->angle;
              angle_error = fabs(tf::getYaw(msg->pose.pose.orientation) * 180 / PI - _targetangle * 180 / PI);
              ROS_INFO_STREAM("angle_error:" << angle_error);
          }


  } else{
      speedCommand = 0;
  }


  //Invoking method for publishing message
  if(angleCommand > maxASpeed){
      angleCommand = maxASpeed;
  } else if(angleCommand < -maxASpeed){
      angleCommand = -maxASpeed;
  }
  publishMessage(angleCommand, fmin(maxSpeed,speedCommand));
}

bool NodePID::anglecloseEnough(MyPoint* actual)
{

    if (fabs(_targetangle - (actual->angle - start->angle)) > toleranceAngle &
        fabs(_targetangle - (actual->angle - start->angle) + 2*PI) > toleranceAngle &
        fabs(_targetangle - (actual->angle - start->angle) - 2*PI) > toleranceAngle)
    {
        return false;
    }
    return true;
}

bool NodePID::discloseEnough(MyPoint* actual)
{
    double distance;
    distance = _target->getDistance(actual);
    if (distance > tolerance)
    {
        return false;
    }
    return true;
}

bool NodePID::closeEnough(MyPoint* actual)
{
  double distance;
  distance = start->getDistance(actual)*copysign(1.0, _targetdistance);
  if (fabs(distance-_targetdistance) > tolerance)
  {
    return false;
  }
  if (fabs(_targetangle - (actual->angle - start->angle)) > toleranceAngle &
    fabs(_targetangle - (actual->angle - start->angle) + 2*PI) > toleranceAngle &
    fabs(_targetangle - (actual->angle - start->angle) - 2*PI) > toleranceAngle)
  {
    return false;
  }
  return true;
}

double NodePID::calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum)
{
    int index;
    double error = reference - actualValue;
  double previousError = reference - lastValue;
  double dt = actual->time.toSec() - last->time.toSec();
  double derivative = (error - previousError)/dt;
  if(pidspeed > maxASpeed){
      if(abs(error) > _mypid.i_param){
          index = 0;
      } else{
          index = 1;
          if(error < 0){
              *sum = *sum + error*dt;
          }
      }
  } else if(pidspeed < -maxASpeed){
      if(abs(error) > _mypid.i_param){
          index = 0;
      } else{
          index = 1;
          *sum = *sum + error*dt;
      }
  } else{
      if(abs(error) > _mypid.i_param){
          index = 0;
      } else{
          index = 1;
          *sum = *sum + error*dt;
      }
  }
    ROS_INFO_STREAM("*sum:" << *sum);
  pidspeed = kP*error + kD*derivative + kS*(*sum);
  return pidspeed;
}
