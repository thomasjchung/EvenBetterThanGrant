#pragma once
#include "main.h"
class robot
{
public:
  static pros::Controller master;
  static pros::Motor LF,LB,RF,RB, flyWheel, flyWheel2;
  static pros::ADIDigitalIn buttonDown, buttonSelect;
  static pros::ADIDigitalOut puncher;
  static pros::IMU imu;
  static pros::Rotation Lencoder,Rencoder,Bencoder;
  static pros::Vision flywheelCam;
};
