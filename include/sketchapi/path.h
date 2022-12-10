#pragma once
#include "main.h"
class path
{
public:
  static double getHeading(std::pair<double,double> p1, std::pair<double,double> p2);
  static std::pair<double,double> getIntersection(std::pair<double,double> p1, std::pair<double,double> p2, std::pair<double,double> currentPos, double radius);
  static double distance(std::pair<double,double> p1, std::pair<double,double> p2);
};
double toRad(double degrees);
