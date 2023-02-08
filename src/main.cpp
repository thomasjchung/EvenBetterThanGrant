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
	robot::piston_expansion.set_value(false);
	robot::piston_launcher.set_value(true);

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
  robot::piston_launcher.set_value(true);
  move.remove();
  set_velocity(-100,0,0);
  pros::delay(1000);
  robot::piston_expansion.set_value(true);
  std::vector<std::pair<double,double>> path = {std::make_pair(1000,0),std::make_pair(500,0)};
  sketch.driveAlong(path, 0, 100, 12);
  std::cout << sketch.robotX << " " << sketch.robotY << std::endl;
  sketch.turn(M_PI / (-4), 5);
  robot::piston_expansion.set_value(false);
  path = {std::make_pair(500,0),std::make_pair(400,-100)};
  sketch.driveAlong(path, 0, 100, 12);
  sketch.turn(M_PI * 0.18, 7);
  
  pros::Task move2(drive2);

  move2.remove();
  pros::delay(250);
  robot::piston_expansion.set_value(true);
  pros::delay(250);
  path = {std::make_pair(1750,1000),std::make_pair(400,-100)};
  sketch.driveAlong(path, M_PI * 0.15, 100, 12);
}

int autonomous_control = 0;

void autonomous() {

	if (autonomous_control == 0){
		
		set_velocity(30,0,0);
		
		//turning the roller
		robot::intake.move_velocity(100);
		pros::delay(350);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		pros::delay(200);
		
		robot::intake.move_velocity(0);
		robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		pros::delay(420); //420 lmfao :rofl: 

		//move back a little?
		set_velocity(-10,0,0);
		pros::delay(100);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		pros::delay(500);

		/*
		//turn
		set_velocity(0,0,-45);
		pros::delay(110);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		pros::delay(500);

		//move towards goal
		set_velocity(-200,0,0);
		pros::delay(1600);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		pros::delay(1000);


		//empty rings from basket
		robot::basket.move_velocity(-100);
		pros::delay(3000);
		robot::basket.move_velocity(0);
		robot::basket.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

		pros::delay(200);

		set_velocity(200,0,0);
		pros::delay(500);
		set_velocity(0,0,0);
		set_brakes(pros::E_MOTOR_BRAKE_BRAKE);
		pros::delay(500);
		*/
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
	int lastPLtoggle = 0;
	int lastPEtoggle = 0;
	bool launcher_on = false;
	bool previously_pressed = false;

	set_brakes(pros::E_MOTOR_BRAKE_COAST);

	std::int32_t launcher_speed{}, launcher_max_speed{400}, launcher_acceleration{50};
	std::int32_t basket_speed{}, basket_max_speed{200}, basket_acceleration{100};
	std::int32_t intake_speed{}, intake_max_speed{500}, intake_acceleration{100};
	std::int32_t expansion_speed{}, expansion_max_speed{200}, expansion_acceleration{10};
	bool is_overheating{}, piston_expansion_state{}, piston_launcher_state{}; //piston_back_lift_state?
	
	while (true) {
		set_velocity(robot::master.get_analog(ANALOG_LEFT_Y),
					0, -(.9*robot::master.get_analog(ANALOG_RIGHT_X)));

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if(basket_speed > -basket_max_speed) {
				basket_speed = std::max(-basket_max_speed, basket_speed - basket_acceleration);
			}
			robot::basket.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::basket.move_velocity(basket_speed);
		} else if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			if(basket_speed < basket_max_speed) {
				basket_speed = std::min(basket_max_speed, basket_speed + basket_acceleration);
			}
			robot::basket.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::basket.move_velocity(basket_speed);
		} else if(basket_speed != 0) {
			if(basket_speed > 0) {
				basket_speed = std::max(basket_speed - basket_acceleration, std::int32_t{});
			} else {
				basket_speed = std::min(basket_speed + basket_acceleration, std::int32_t{});
			}
			robot::basket.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::basket.move_velocity(basket_speed);
		} else {
			robot::basket.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			robot::basket.move_velocity(basket_speed);
		}


		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::intake.move_velocity(-intake_max_speed);
		} else if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::intake.move_velocity(intake_max_speed);
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
		
		
		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && pros::millis() - lastPEtoggle > 750) {     //pros::E_CONTROLLER_DIGITAL_UP
			lastPEtoggle = pros::millis();
			piston_expansion_state = !piston_expansion_state;
			robot::piston_expansion.set_value(piston_expansion_state);
		}

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && pros::millis() - lastPLtoggle > 750) {
			lastPLtoggle = pros::millis();
			piston_launcher_state = !piston_launcher_state;
			robot::piston_launcher.set_value(piston_launcher_state);
		}

		/*
		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(!previously_pressed){
				launcher_on = !launcher_on;
			}
			previously_pressed = true;
		}
		else{
			previously_pressed = false;
		}

		if(launcher_on){
			if(launcher_speed > -launcher_max_speed) {
				launcher_speed = std::max(-launcher_max_speed, launcher_speed - launcher_acceleration);
			}
			robot::launcher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::launcher.move_velocity(-launcher_speed);		}
		else{
			robot::launcher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		*/

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			if(launcher_speed > -launcher_max_speed){
				launcher_speed = std::max(-launcher_max_speed, launcher_speed - launcher_acceleration);
			}
			robot::launcher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::launcher.move_velocity(launcher_speed);
		}
		else{
			robot::launcher.move_velocity(0);
			robot::launcher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		

		if(robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			robot::expansion.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::expansion.move_velocity(-expansion_max_speed);
		} else if(expansion_speed != 0) {
			if(expansion_speed > 0) {
				expansion_speed = std::max(expansion_speed - expansion_acceleration, std::int32_t{});
			} else {
				expansion_speed = std::min(expansion_speed + expansion_acceleration, std::int32_t{});
			}
			robot::expansion.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			robot::expansion.move_velocity(expansion_speed);
		} else {
			robot::expansion.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			robot::expansion.move_velocity(expansion_speed);
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
