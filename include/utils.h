#pragma once
#include "main.h"
void setChassisBrakes(const pros::motor_brake_mode_e brakeMode);
void setChassisVelocity(double straight, double strafe, double turn);
double round_to_digits(double value, int digits);
double decimalMod(double value, double base);
void reset_encoders();
