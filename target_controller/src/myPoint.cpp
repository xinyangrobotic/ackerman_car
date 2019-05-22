#include "myPoint.h"

MyPoint::MyPoint(double xPos, double yPos, double alpha, int turn_around, ros::Time t)
{
  x = xPos;
  y = yPos;
  angle = alpha;
  time = t;
  turn = turn_around;
}

MyPoint::~MyPoint()
{
}

double MyPoint::getAngle(MyPoint* target)
{
  return atan2((target->y - y),(target->x - x));
}

double MyPoint::getDistance(MyPoint* target) {
    return sqrt((pow(target->y - y, 2.0)) + (pow(target->x - x, 2.0)));
}
