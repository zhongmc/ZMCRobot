#ifndef _VELOCITY_CONTROLLER_H_
#define _VELOCITY_CONTROLLER_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class VelocityController : public Controller
{
public:
  VelocityController();
  void reset();
  void execute(Robot *robot, Input *input, Output *output, double dt);
  void setGoal(double v, double theta, double curTheta);

private:
  double mTheta; //目标方向
  double mW;     //转弯
};

#endif /* _VELOCITY_CONTROLLER_H_ */
