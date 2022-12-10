#include "robot.hpp"

pros::Controller robot::master (pros::E_CONTROLLER_MASTER);

pros::Motor robot::motor_lf(5, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_lb(17, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_rf(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::motor_rb(19, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::fourbar_l(12, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_COUNTS), // cable for 11 may be bad
            robot::fourbar_r(15, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::frontclaw(3, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::intake(14, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS),
            robot::goofyArm(3, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

pros::ADIDigitalOut robot::piston_front_claw('B'),
                    robot::piston_back_claw('H');     //piston_back_lift?
                 //we DONT NEED THEM ANYMORE?????  - jeff FUCK OFF JEFF

pros::Rotation robot::encoder_l(13),
               robot::encoder_r(8),
               robot::encoder_b(16);

double robot::encoder_diameter(3.25),
       robot::side_encoder_r(6.82367),
       robot::back_encoder_r(4);

pros::IMU robot::imu(13);
