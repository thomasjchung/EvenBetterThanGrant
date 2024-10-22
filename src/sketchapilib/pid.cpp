#include "sketchapilib/pid.h"
#include "main.h"
#include <cmath>

PID::PID(double p, double i, double d, double min, double max)
{
  kP = p;
  kI = i;
  kD = d;
  minSpeed = min;
  maxSpeed = max;
  ticks = 0;
  priorTime = 0;
  priorError = 0;
  integral = 0;
}

double PID::tick(double error)
{
  if(error == 0)
  {
    return 0;
  } 
  double errorDerivative = (error - priorError);
  integral += ((error + priorError) / 2);
  double speed = (kP * error) + ((ticks != 0) ? (kI * integral) + (kD * errorDerivative) : 0);

  priorError = error;
  ticks++;
  double output = std::max(minSpeed,std::abs(speed)) * (speed / std::abs(speed));
  if(output > maxSpeed)
  {
    return maxSpeed;
  }
  else
  {
    return output;
  }
}

void PID::reset()
{
  integral = 0;
  ticks = 0;
}
