#pragma once
#include <string>
class autonSelector
{
public:
  autonSelector();
  int run();
  void updateDisplay(int current);
  int numAutons = 5;
};
