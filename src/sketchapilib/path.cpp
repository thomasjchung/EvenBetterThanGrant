#include <cmath>
#include "sketchapilib/path.h"

int sign(double x)//returns the sign of the inputted integer
{
  if (x >= 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

double toRad(double degrees)
{
  return (degrees * M_PI) / 180;
}

double path::getHeading(std::pair<double,double> p1, std::pair<double,double> p2)//gets the heading the robot has to drive at to go from point 1 to point 2
{
  return atan2(p2.second-p1.second,p1.first-p2.first);
}

double path::distance(std::pair<double,double> p1, std::pair<double,double> p2)//distance formula
{
  return sqrt(std::pow(p1.first-p2.first,2)+std::pow(p1.second-p2.second,2));
}

std::pair<double,double> path::getIntersection(std::pair<double,double> start, std::pair<double,double> end, std::pair<double,double> cur, double radius) 
{
  std::pair<double,double> p1  = std::make_pair(start.first - cur.first, start.second - cur.second);
  std::pair<double,double> p2  = std::make_pair(end.first - cur.first, end.second - cur.second);

  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  float d = sqrt(std::pow(dx,2) + std::pow(dy,2));
  float D = p1.first * p2.second - p2.first * p1.second;
  float discriminant = std::pow(radius,2) * std::pow(d,2) - std::pow(D,2);

  float x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / std::pow(d,2);
  float y1 = (-D * dx + std::abs(dy) * sqrt(discriminant)) / std::pow(d,2);
  
  float x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / std::pow(d,2);
  float y2 = (-D * dx - std::abs(dy) * sqrt(discriminant)) / std::pow(d,2);
  //Above calculations came from here: https://mathworld.wolfram.com/Circle-LineIntersection.html

  std::pair<double,double> intersection1 = std::make_pair(x1, y1);
  std::pair<double,double> intersection2 = std::make_pair(x2, y2);

  float distance1 = distance(p2, intersection1);
  float distance2 = distance(p2, intersection2);

  std::pair<double,double> calc1 = std::make_pair((x1 + cur.first), (y1 + cur.second));
  std::pair<double,double> calc2 = std::make_pair((x2 + cur.first), (y2 + cur.second));

  if (distance1 <= distance2) 
  {
    return calc1;
  }
  else 
  {
    return calc2;
  }
}

