#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "main.h"
#include "utils.hpp"

struct robot {
    static pros::Controller master;
    static pros::Motor motor_lf, motor_lb, motor_rf, motor_rb, motor_lm, motor_rm, launcher, basket, intake, expansion;
    static pros::ADIDigitalOut piston_expansion, piston_launcher;  //piston_back_lift?
    static pros::Rotation encoder_l, encoder_r, encoder_b;
    static double encoder_diameter, side_encoder_r, back_encoder_r;
    static pros::IMU imu;
};

void move(double dx, double dy, double dtheta);

#endif