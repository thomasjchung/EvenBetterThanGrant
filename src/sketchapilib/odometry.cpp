#include "sketchapilib/sketchapi.h"
#include "robot.hpp"
#include "utils.hpp"
#include "sketchapilib/path.h"
#include <cmath>

sketchapi::sketchapi(double bER, double sER, double eWC)
{
  turnPID.reset();
  xPID.reset();
  yPID.reset();
  robotX = 0;
  robotY = 0;
  xTurnOffset = 0;
  robotPhi = 0;
  priorPhi = 0;
  backEncoderR = bER;
  sideEncoderR = sER;
  encoderWheelCircumference = eWC;
}
void sketchapi::reset()
{
  reset_encoders();
  robotX = 0;
  robotY = 0;
  xTurnOffset = 0;
  robotPhi = 0;
  priorPhi = 0;
  turnPID.reset();
  xPID.reset();
  yPID.reset();
  //robot::imu.tare();
}
void sketchapi::odometry()
{
  reset();
  double phi;
  double dPhi;

  double isolatedX;
  double dx;
  double priorx = 0;

  double isolatedY;
  double dy;
  double priory = 0;

  double dyTurnOffset;
  double dxTurnOffset;

  int loops = -1;
  while(true)
  {
    phi = toRad(round_to_digits(robot::imu.get_rotation(),2)) - M_PI;
    dPhi = phi - priorPhi;

    if(dPhi > M_PI)
    {
      dPhi -= 2 * M_PI;
    }
    else if(dPhi < -M_PI)
    {
      dPhi += 2 * M_PI;
    }

    dxTurnOffset = 360 * (backEncoderR * dPhi) / encoderWheelCircumference;
    xTurnOffset += dxTurnOffset;

    isolatedX = (round_to_digits(robot::encoder_l.get_position(),4) / 100 + round_to_digits(robot::encoder_r.get_position(),4) / 100) / 2;
    isolatedY = round_to_digits(robot::encoder_b.get_position(),2) / 100 + xTurnOffset;

    dx = isolatedX - priorx;
    dy = isolatedY - priory;

    robotPhi = -(phi - (std::floor(phi / 6.28318) * 6.28318) - M_PI);
    robotY += dy * std::cos(robotPhi) + dx * std::sin(robotPhi);
    robotX += dx * std::cos(robotPhi) - dy * std::sin(robotPhi);

    priorPhi = phi;
    priorx = isolatedX;
    priory = isolatedY;

    loops++;
    if(loops % 10 == 0)
    {
      //std::cout << "  " << robotPhi << "  " << robotX << "  " << robotY << " " << round_to_digits(robot::Lencoder.get_position(),4) / 100 << " " << round_to_digits(robot::encoder_r.get_position(),4) / 100 << std::endl;
    }
    pros::delay(30);
  }
}