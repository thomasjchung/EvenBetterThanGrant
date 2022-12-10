#include "sketchapilib/sketchapi.h"
#include "sketchapilib/path.h"
#include "robot.hpp"
#include "utils.hpp"
#include <cmath>

path Path = path(); // make an instance of the path class

bool sketchapi::updateChassisPID(std::pair<double, double> destination, double desiredPhi, double end)
{
  double xOut = xPID.tick(destination.first - robotX);
  double yOut = yPID.tick(destination.second - robotY);
  double tOut = turnPID.tick(decimalMod(desiredPhi - robotPhi, M_PI));

  double straightVelocity = yOut*std::sin(robotPhi)+xOut*std::cos(robotPhi);
  double strafeVelocity = 0/*xError*std::sin(robotPhi)-yError*std::cos(robotPhi)*/;
  if(straightVelocity < 0)
  {
    straightVelocity*=0.5;
  }
  set_velocity(straightVelocity,strafeVelocity,-tOut);
  return (std::abs(straightVelocity) < end && std::abs(strafeVelocity) < end && std::abs(tOut) < end);
}

void sketchapi::turn(double phi, double end)
{
  turnPID2.reset();
  bool done = false;
  while(!done)
  {
    double out = turnPID2.tick(decimalMod(robotPhi - phi, 2*M_PI));
    done = std::abs(out) < end;
    set_velocity(0,0,out);
  }
  set_velocity(0,0,0);
  set_brakes(pros::E_MOTOR_BRAKE_HOLD);
}

void sketchapi::driveAlong(std::vector<std::pair<double,double>> points, double endPhi, double radius, double exit)
{
  turnPID.reset();
  xPID.reset();
  yPID.reset();
  std::pair<double, double> start;
  std::pair<double, double> end;
  std::pair<double, double> target;

  std::cout << "start drivealong";

  for(int index = 0; index < (points.size() - 1); index++)
  {
    start = points[index];
    end = points[index+1];
    bool done = false;
    while((Path.distance(std::make_pair(robotX,robotY), end) > radius && !(index == (points.size() - 2))) || (index == (points.size() - 2) && !done))
    {
      if((index == points.size() - 2) && (Path.distance(std::make_pair(robotX,robotY), end) > radius))
      {
        target = end;
        done = updateChassisPID(target,endPhi,exit);
      }
      else
      {
        target = Path.getIntersection(start, end, std::make_pair(robotX,robotY),radius*2);
        if(isnan(target.first) || isnan(target.second))
        {
         target = start;
        }
        done = updateChassisPID(target,Path.getHeading(std::make_pair(robotX, robotY), target),exit);
      }
      pros::delay(30);
    }
  }
  set_velocity(0,0,0);
  set_brakes(pros::E_MOTOR_BRAKE_HOLD);

  std::cout << "end drivealong";
}
