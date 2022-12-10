#pragma once
#include "pid.h"
#include "main.h"
#include <vector>
class sketchapi
{
public:
  // backEncoderRadius, sideEncoderRadius, encoderWheelCircumference
  sketchapi(double bER, double sER, double eWC);
  void odometry();
  void reset();
  void driveAlong(std::vector<std::pair<double,double>> path, double endPhi, double radius, double end);
  void turn(double phi, double end);
  void xdrive(double x, double end);

  double robotX;
  double robotY;
  double xTurnOffset;
  double priorPhi;
  double robotPhi;
  PID xPID = PID(0.25,0,0.35,0,200);
  PID yPID = PID(0.27,0,0.35,0,200);
private:
  bool updateChassisPID(std::pair<double, double> destination, double desiredPhi, double end);
  PID turnPID2 = PID(50,0,10,0,200);
  PID turnPID = PID(40,0,0,0,200);
  PID fourBarPID = PID(0.32,0,0.6,0,200);
  PID backLiftPID = PID(0.42,0,0.6,0,200);
  double backEncoderR;
  double sideEncoderR;
  double encoderWheelCircumference;
};
