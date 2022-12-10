#ifndef UTILS_HPP
#define UTILS_HPP

#include "main.h"
#include "robot.hpp"

double threshold_normalize(const double& input, const std::uint8_t& threshold = 10);

void set_brakes(const pros::motor_brake_mode_e& brakeMode);

void set_velocity(const double& straight, const double& strafe, const double& rotate);
double round_to_digits(double value, int digits);
double decimalMod(double value, double base);

void reset_encoders();
double averageIMU();


#endif