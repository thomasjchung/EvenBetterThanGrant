#include "robot.hpp"

pros::Controller robot::master (pros::E_CONTROLLER_MASTER);

pros::Motor robot::motor_lf(16, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_lb(13, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_rf(3, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_rb(18, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_lm(12, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_rm(1,MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::launcher(19, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::basket(5, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::intake(20, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::expansion(17, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

pros::ADIDigitalOut robot::piston_expansion('B'),
                    robot::piston_launcher('H');     //piston_back_lift?
                 //we DONT NEED THEM ANYMORE?????  - jeff FUCK OFF JEFF

pros::Rotation robot::encoder_l(13),
               robot::encoder_r(8),
               robot::encoder_b(6);

double robot::encoder_diameter(3.25),
       robot::side_encoder_r(6.82367),
       robot::back_encoder_r(4);

pros::IMU robot::imu(13);
