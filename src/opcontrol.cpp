#include "robot.h"
#include "main.h"
#include "utils.h"
#include <cmath>
#include <vector>
#include "sketchapi/sketchapi.h"

#define forwardBackwardJoystick ANALOG_RIGHT_Y
#define sideToSideJoystick ANALOG_RIGHT_X
#define rotateJoystick ANALOG_LEFT_X

#define driveModeToggle DIGITAL_DOWN
#define driveModeToggleDelay 500

#define puncherButton DIGITAL_UP
#define puncherDelay 200

#define flywheelButton1 DIGITAL_R1
#define flywheelButton2 DIGITAL_R2
#define flywheelButton3 DIGITAL_L1
#define flywheelButton4 DIGITAL_L2
#define flywheelButtonAuto DIGITAL_X

bool overheat = false;

int numPunches = 0;

int priorTime = pros::millis();
void puncherControl()
{
    if(robot::master.get_digital(puncherButton)) // if the puncher button is pressed, note the time and extend the puncher
    {
        priorTime = pros::millis();
        robot::puncher.set_value(1);
    }
    if(pros::millis() - priorTime > puncherDelay) // wait to release the puncher so that it doesn't get activated multiple times from a single press
    {
        numPunches++;
        robot::puncher.set_value(0);
    }
}

int flyWheelVel = 0;
PID flywheelPID = PID(0,0.1,0.5,-12000,12000);

void flywheelControl()
{
    if(robot::master.get_digital(flywheelButton1)) // use the 4 triggers on the controller for different flywheel speeds for testing
    {
        flyWheelVel = 360;
    }
    else if(robot::master.get_digital(flywheelButton2))
    {
        flyWheelVel = 340;
    }
    else if(robot::master.get_digital(flywheelButton3))
    {
        flyWheelVel = 320;
    }
    else if(robot::master.get_digital(flywheelButton4))
    {
        flyWheelVel = 300;
    }
    else if(robot::master.get_digital(flywheelButtonAuto))
    {
        flyWheelVel = 340 - (robot::flywheelCam.get_by_sig(0, 1).width / 2) + numPunches;
    }
    else
    {
        flyWheelVel = 0;
    }


    // update the pid loop for the flywheel with the current error and feed the output into the speeds of the motors
    double flyWheelPIDError = 2 * flyWheelVel - robot::flyWheel.get_actual_velocity() - robot::flyWheel2.get_actual_velocity();
    if(isinf(flyWheelPIDError))
    {
        flyWheelPIDError = 0;
        robot::master.rumble(". ");
    }
    robot::master.set_text(0, 0, "        ");
    robot::master.set_text(0, 0, flyWheelPIDError);
    int flyWheelPIDout = flywheelPID.tick(flyWheelPIDError);
    robot::flyWheel.move_voltage(flyWheelPIDout);
    robot::flyWheel2.move_voltage(flyWheelPIDout);
}

bool trackingGoal = false;
int priorToggleTime = pros::millis();
PID goalTrackPID = PID(0.2,0,0,-50,50);

void chassisControl()
{
    if(robot::master.get_digital(driveModeToggle) && pros::millis() - priorToggleTime > driveModeToggleDelay) // button to toggle between chassis modes
    {
        priorToggleTime = pros::millis();
        trackingGoal = !trackingGoal;
        if(trackingGoal)
        {
            goalTrackPID.reset();
        }
    }

    if(!trackingGoal) // if it isn't tracking, update everything based on joysticks
    {
        setChassisVelocity(robot::master.get_analog(forwardBackwardJoystick),
                           robot::master.get_analog(sideToSideJoystick),
                           robot::master.get_analog(rotateJoystick) /1.5);
    }
    else // otherwise, update translational speeds from joysticks but rotational the output of the pid which is given vision sensor data as its error
    {
        setChassisVelocity(robot::master.get_analog(forwardBackwardJoystick),
                           robot::master.get_analog(sideToSideJoystick),
                           goalTrackPID.tick(robot::flywheelCam.get_by_sig(0, 1).x_middle_coord-10));
    }
}

void overheatCheck()
{
    // if any motors are overheated vibrate the controller
    if(robot::LF.is_over_temp() || robot::LB.is_over_temp() || robot::RF.is_over_temp() || robot::RB.is_over_temp() || robot::flyWheel.is_over_temp() || robot::flyWheel2.is_over_temp())
    {
        if(!overheat)
        {
            robot::master.rumble(" . . . ");
            overheat = true;
        }
    }
    else
    {
        overheat = false;
    }
}