#include "robot.h"

pros::Controller robot::master (pros::E_CONTROLLER_MASTER);

pros::Motor robot::LF(3, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::LB(17, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::RF(9, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::RB(18, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            robot::flyWheel(21, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            robot::flyWheel2(10, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS);

pros::ADIDigitalIn robot::buttonDown('a'),
                   robot::buttonSelect('b');

pros::ADIDigitalOut robot::puncher('h');

pros::Rotation robot::Lencoder(13),
               robot::Rencoder(12),
               robot::Bencoder(20);

pros::IMU robot::imu(10);

pros::Vision robot::flywheelCam(5);