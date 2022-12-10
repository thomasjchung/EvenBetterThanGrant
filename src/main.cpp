#include "main.h"
#include "robot.h"
#include "utils.h"
#include "opcontrol.h"
#include "autonselector.h"

#include "sketchapi/sketchapi.h"

#include <iostream>

int selectedAutonomous = 0;
autonSelector selector = autonSelector(); // init the auton selector

sketchapi Sketch = sketchapi(3.125, 3.125, 8.57);

void odomFunction(void* param) // creates a function we can use to create a thread
{
	Sketch.odometry();
}

void initialize() 
{
	std::cout << "initialize routine" << std::endl;

  robot::puncher.set_value(0); // make sure the puncher isn't extended

  robot::flywheelCam.set_zero_point(pros::E_VISION_ZERO_CENTER); // initialize vision sensor color signatures from the vision utility
  pros::vision_signature_s_t RED_SIG = pros::c::vision_signature_from_utility(1, 8099, 8893, 8496, -1505, -949, -1227, 9.5, 0);
  robot::flywheelCam.set_signature(1, &RED_SIG);

	pros::lcd::initialize(); // initialize lcd and print Sketch to the screen
	pros::lcd::set_text(1, "Sketch");

	pros::delay(2000);

	std::cout << "initialize routine complete" << std::endl;
}

void disabled() {}

void competition_initialize() 
{
	std::cout << "competition initialize routine" << std::endl;

	selectedAutonomous = selector.run(); // run the auton selector

	std::cout << "competition initialize routine complete" << std::endl;
}

void opcontrol() 
{
  //pros::Task odomTask (odomFunction, (void*)"", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odomTask");
	//Sketch.reset();

	while (true) 
	{
    flywheelControl();
    puncherControl();
    chassisControl();
    overheatCheck();

    pros::delay(20);
	}
}

void right()
{
}

void right2()
{
}

void left()
{
}

void left2()
{
}

void skills()
{
}

void autonomous() 
{
  switch(selectedAutonomous)
  {
    case 0:
      right();
      break;
    case 1:
      right2();
      break;
    case 2:
      left();
      break;
    case 3:
      left2();
      break;
    case 4:
      skills();
      break;
  }
}
