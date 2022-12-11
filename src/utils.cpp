#include "utils.hpp"

double threshold_normalize(const double& input, const std::uint8_t& threshold) {
	return input;
	if(input > threshold) {
		return input - threshold;
	} else if(input < -threshold) {
		return input + threshold;
	} else {
		return 0;
	}
}

void set_brakes(const pros::motor_brake_mode_e& brakeMode) {
  robot::motor_lf.set_brake_mode(brakeMode);
	robot::motor_lb.set_brake_mode(brakeMode);
	robot::motor_rf.set_brake_mode(brakeMode);
	robot::motor_rb.set_brake_mode(brakeMode);
}

// https://www.youtube.com/watch?v=v7CujEW0wgc  
void set_velocity(const double& straight, const double& strafe, const double& rotate) {
	  robot::motor_lf.move_velocity((straight + strafe - rotate) * 200 / 127);
    robot::motor_rf.move_velocity((-straight + strafe - rotate) * 200 / 127);
  	robot::motor_lb.move_velocity((straight - strafe - rotate) * 200 / 127);
  	robot::motor_rb.move_velocity((-straight - strafe - rotate) * 200 / 127);
}

void reset_encoders()
{
  robot::encoder_b.reset_position();
  robot::encoder_l.reset_position();
  robot::encoder_r.reset_position();
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