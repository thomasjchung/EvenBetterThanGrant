#include "autonSelector.h"
#include "main.h"
#include "robot.h"
autonSelector::autonSelector(){} // it isn't necessary to do anything when the class is initialized
int autonSelector::run() // when the auton selector is runned (most likely in competition_initialize)
{
  bool notDone = true;
  int currentSelected = 0;
  updateDisplay(currentSelected);
  std::cout << "running auton selector" << std::endl;
  while(notDone)
  {
    if(robot::buttonDown.get_value()) // certain button is pressed
    {
      currentSelected++; // increment the selected val
      if(currentSelected == numAutons)
      {
        //currentSelected--; // reverse the incrementing if we got to the end of the list
        currentSelected = 0; // go to start wif we got to the end
      }
      updateDisplay(currentSelected); // update display
    }
    /*else if(robot::buttonUp.get_value())
    {
      currentSelected--; // decrement the selected val
      if(currentSelected < 0)
      {
        currentSelected = 0; // don't let it go negative
      }
      updateDisplay(currentSelected);
    }*/
    else if(robot::buttonSelect.get_value()) // finalize button
    {
      notDone = false;
      std::cout << "confirmed: " << currentSelected << std::endl;
      pros::lcd::set_text(numAutons+1,"confirmed"); // confirmation message on screen
    }
    pros::delay(150); // avoid brain lag and also so it doesn't register as many double presses, making it easier to select a specific auton
  }
  return currentSelected; // return the selected auton
}

std::string autonNames[] = {"R","R2","L","L2","skills"};

void autonSelector::updateDisplay(int current)
{
  std::string toPrint = "";
  std::string currentName;
  for(int i = 0; i < numAutons; i++)
  {
    if(i == current)
    {
      currentName = " >" + autonNames[i] + "< ";
    }
    else
    {
      currentName = " " + autonNames[i] + " ";
    }
    toPrint += currentName;
  }
  pros::lcd::set_text(0, toPrint);
}
