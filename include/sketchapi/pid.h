#pragma once
class PID
{
public:
  double kP;
  double kI;
  double kD;
  PID(double p, double i, double d, double min, double max);
  double tick(double error);
  void reset();
  double minSpeed;
  double maxSpeed;
private:
  double ticks;
  double priorError;
  double priorTime;
  double integral;
};
