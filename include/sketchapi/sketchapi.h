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

  double robotX;
  double robotY;
  double xTurnOffset;
  double yTurnOffset;
  double priorPhi;
  double robotPhi;
  PID xPID = PID(0.25,0,0.35,0,200);
  PID yPID = PID(0.27,0,0.35,0,200);
  PID turnPID = PID(40,0,0,0,200);
private:
  double backEncoderR;
  double sideEncoderR;
  double encoderWheelCircumference;
};
