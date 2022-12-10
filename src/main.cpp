#include "robot.hpp"
#include "main.h"
#include "sketchapilib\sketchapi.h"
#include "utils.hpp"
#include <vector>

#define forwardBackwardJoystick ANALOG_RIGHT_Y
#define sideToSideJoystick ANALOG_RIGHT_X
#define rotateJoystick ANALOG_LEFT_X


sketchapi sketch = sketchapi(0,6.75, 3.25 * 3.14159); // init sketchapilib - has the pid, path, odom, and move functions

void odometry_function(void* parameter) {
	sketch.odometry();
}

void initialize() {
	pros::lcd::initialize();
	robot::piston_front_claw.set_value(true);
	robot::piston_back_claw.set_value(true);

	robot::encoder_l.reset();
	robot::encoder_r.reset();
	robot::encoder_b.reset();
	  
	sketch.reset();
}
void disabled() {}

void competition_initialize() {

}

// strafe 235
// straight 235
// theta degrees

void drive1()
{
  std::cout << "do drive 1";
  std::vector<std::pair<double,double>> path = {std::make_pair(0,0),std::make_pair(1700,0)};
  sketch.driveAlong(path, 0, 100, 10);
  std::cout << "finish drive 1";
}

void drive2()
{
  std::vector<std::pair<double,double>> path = {std::make_pair(400,-100),std::make_pair(1750,800)};
  sketch.driveAlong(path, M_PI * 0.15, 100, 5);
  set_velocity(30,0,0);
}
void right()
{
  sketch.reset();
  pros::Task move(drive1);
  robot::piston_back_claw.set_value(true);
  move.remove();
  set_velocity(-100,0,0);
  pros::delay(1000);
  robot::piston_front_claw.set_value(true);
  std::vector<std::pair<double,double>> path = {std::make_pair(1000,0),std::make_pair(500,0)};
  sketch.driveAlong(path, 0, 100, 12);
  std::cout << sketch.robotX << " " << sketch.robotY << std::endl;
  sketch.turn(M_PI / (-4), 5);
  robot::piston_front_claw.set_value(false);
  path = {std::make_pair(500,0),std::make_pair(400,-100)};
  sketch.driveAlong(path, 0, 100, 12);
  sketch.turn(M_PI * 0.18, 7);
  
  pros::Task move2(drive2);

  move2.remove();
  pros::delay(250);
  robot::piston_front_claw.set_value(true);
  pros::delay(250);
  path = {std::make_pair(1750,1000),std::make_pair(400,-100)};
  sketch.driveAlong(path, M_PI * 0.15, 100, 12);
}

int autonomous_control = 0;
// 1 two left
// 2 middle left
// 3 two right
// 4 middle right
// 5 skills
void autonomous() {
	robot::piston_front_claw.set_value(false);
	robot::piston_back_claw.set_value(false);
	if (autonomous_control == 0){
		robot::goofyArm.move_velocity(100);   //arm moves down
		set_velocity(-200,0,0);   //robot starts moving toward goal
		pros::delay(400);
		robot::goofyArm.move_velocity(0);
		robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::delay(420);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_COAST);

		pros::delay(400);   //get there and stop

		robot::goofyArm.move_velocity(-100);   //arm moves slightly up to hook
		pros::delay(250);  
		set_velocity(200,0,0);   					//moves back towards ramp
		robot::goofyArm.move_velocity(0);
		robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::delay(700);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_COAST);

		pros::delay(1000);  		//get there and stop (hopefully with goal)

		robot::goofyArm.move_velocity(100);  		//arm moves down a little)
		pros::delay(50);
		robot::goofyArm.move_velocity(0);
		robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::delay(400);

		set_velocity(50, 0, 0);  		//robot moves forward a little
		pros::delay(30);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);

		robot::goofyArm.move_velocity(-100);      //arm goes back up
		pros::delay(400);
		robot::goofyArm.move_velocity(0);
		robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::delay(400);

		set_velocity(0,0, -90);   //rotate?
		pros::delay(150);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		


		




	}

}

// mechanum diameter
// mechanum track size
// tracking wheel diameter
// tracking wheel track
// middle wheel distance
// 2 in middle from bottom

// too fast decrease p
// too slow increase p
// not there increase i
// overshoot decrease p and increase d by little


void opcontrol() {
	int lastFCtoggle = 0;
	int lastBCtoggle = 0;

	set_brakes(pros::E_MOTOR_BRAKE_COAST);

	std::int32_t fourbar_speed{}, fourbar_max_speed{200}, fourbar_acceleration{50};
	std::int32_t intake_speed{}, intake_max_speed{200}, intake_acceleration{100};
	std::int32_t goofyArm_speed{}, goofyArm_max_speed{45}, goofyArm_acceleration{10};
	bool is_overheating{}, piston_front_claw_state{}, piston_back_claw_state{}; //piston_back_lift_state?
	
	while (true) {
		set_velocity(-threshold_normalize(robot::master.get_analog(ANALOG_LEFT_Y)),
						0,
					 -threshold_normalize(-robot::master.get_analog(ANALOG_RIGHT_X)));

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			if(fourbar_speed < fourbar_max_speed) {
				fourbar_speed = std::min(fourbar_max_speed, fourbar_speed + fourbar_acceleration);
			}
			robot::fourbar_l.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_r.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_l.move_velocity(fourbar_speed);
			robot::fourbar_r.move_velocity(fourbar_speed);
		} else if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			if(fourbar_speed > -fourbar_max_speed) {
				fourbar_speed = std::max(-fourbar_max_speed, fourbar_speed - fourbar_acceleration);
			}
			robot::fourbar_l.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_r.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_l.move_velocity(fourbar_speed);
			robot::fourbar_r.move_velocity(fourbar_speed);
		} else if(fourbar_speed != 0) {
			if(fourbar_speed > 0) {
				fourbar_speed = std::max(fourbar_speed - fourbar_acceleration, std::int32_t{});
			} else {
				fourbar_speed = std::min(fourbar_speed + fourbar_acceleration, std::int32_t{});
			}
			robot::fourbar_l.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_r.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::fourbar_l.move_velocity(fourbar_speed);
			robot::fourbar_r.move_velocity(fourbar_speed);
		} else {
			robot::fourbar_l.move_velocity(fourbar_speed);
			robot::fourbar_r.move_velocity(fourbar_speed);
			robot::fourbar_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			robot::fourbar_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			if(intake_speed < intake_max_speed) {
				intake_speed = std::min(intake_max_speed, intake_speed + intake_acceleration);
			}
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::intake.move_velocity(intake_speed);
		} else if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if(intake_speed > -intake_max_speed) {
				intake_speed = std::max(-intake_max_speed, intake_speed - intake_acceleration);
			}
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::intake.move_velocity(intake_speed);
		} else if(intake_speed != 0) {
			if(intake_speed > 0) {
				intake_speed = std::max(intake_speed - intake_acceleration, std::int32_t{});
			} else {
				intake_speed = std::min(intake_speed + intake_acceleration, std::int32_t{});
			}
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::intake.move_velocity(intake_speed);
		} else {
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			robot::intake.move_velocity(intake_speed);
		}
		
		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && pros::millis() - lastFCtoggle > 750) {     //pros::E_CONTROLLER_DIGITAL_UP
			lastFCtoggle = pros::millis();
			piston_front_claw_state = !piston_front_claw_state;
			robot::piston_front_claw.set_value(piston_front_claw_state);
		}

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && pros::millis() - lastBCtoggle > 750) {
			lastBCtoggle = pros::millis();
			piston_back_claw_state = !piston_back_claw_state;
			robot::piston_back_claw.set_value(piston_back_claw_state);
		}
		/*
		else if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			piston_back_claw_state = !piston_back_claw_state;
			robot::piston_back_claw.set_value(piston_back_claw_state);
			pros::delay(500);
		}
		*/
		
		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			if(goofyArm_speed < goofyArm_max_speed) {
				goofyArm_speed = std::min(goofyArm_max_speed, goofyArm_speed + goofyArm_acceleration);
			}
			robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::goofyArm.move_velocity(goofyArm_speed);
		} else if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			if(goofyArm_speed > -goofyArm_max_speed) {
				goofyArm_speed = std::max(-goofyArm_max_speed, goofyArm_speed - goofyArm_acceleration);
			}
			robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::goofyArm.move_velocity(goofyArm_speed);
		} else if(goofyArm_speed != 0) {
			if(goofyArm_speed > 0) {
				goofyArm_speed = std::max(goofyArm_speed - goofyArm_acceleration, std::int32_t{});
			} else {
				goofyArm_speed = std::min(goofyArm_speed + goofyArm_acceleration, std::int32_t{});
			}
			robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::goofyArm.move_velocity(goofyArm_speed);
		} else {
			robot::goofyArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			robot::goofyArm.move_velocity(goofyArm_speed);
		}

		/*
		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			odometry.reset();
		}
		*/

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			autonomous();
		}
		
		//MAKE SURE TO COMMENT OUT ABOVE WHEN JEFF  IS ZOOM ZOOMING ON FIELD DURING COMP UH TITION

		if(robot::motor_lf.is_over_temp() || robot::motor_lb.is_over_temp() || robot::motor_rf.is_over_temp() || robot::motor_rb.is_over_temp()) {
			if(!is_overheating) {
				robot::master.rumble(" . . . ");
				is_overheating = true;
			}
		} else {
			is_overheating = false;
		}

		pros::delay(30);
	}
}
