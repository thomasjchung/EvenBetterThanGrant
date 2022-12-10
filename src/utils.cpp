#include "robot.h"
#include "utils.h"
#include <cmath>

void setChassisBrakes(const pros::motor_brake_mode_e brakeMode) // makes it easier to set the mode of all of the chassis motor's brakes at once
{
  robot::LF.set_brake_mode(brakeMode);
	robot::LB.set_brake_mode(brakeMode);
	robot::RF.set_brake_mode(brakeMode);
	robot::RB.set_brake_mode(brakeMode);
}

void setChassisVelocity(double straight, double strafe, double turn) // set the motor velocities based on the straight, strafe, and turn parts
{
  robot::LF.move_velocity((straight + strafe + turn) * 1.575); // 1.575 is 200/127
  robot::LB.move_velocity((straight - strafe + turn) * 1.575);
  robot::RF.move_velocity((-straight + strafe + turn) * 1.575);
  robot::RB.move_velocity((-straight - strafe + turn) * 1.575);
}

double round_to_digits(double value, int digits)
{
    if (value == 0.0) // otherwise it will return 'nan' due to the log10() of zero
        return 0.0;

    double factor = pow(10.0, digits - ceil(log10(fabs(value))));
    return round(value * factor) / factor;   
}

double decimalMod(double value, double base)
{
  while(std::abs(value) > (base / 2)) // pick which direction we are closer to so we turn in the better direction
  {
    if(value < 0)
    {
      value += base;
    }
    else
    {
      value -= base;
    }
  }
  return value;
}

void reset_encoders()
{
  robot::Bencoder.reset_position();
  robot::Lencoder.reset_position();
  robot::Rencoder.reset_position();
}