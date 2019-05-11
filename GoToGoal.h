#ifndef _GOTOGOAL_H_
#define _GOTOGOAL_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class GoToGoal : public Controller
{
public:
  GoToGoal();
  void reset();
  void execute(Robot *robot, Input *input, Output *output, double dt);

  void setPID(int type, double kp, double ki, double kd);
  //    private:
  //        double fuckEk_l;
  //        double E_k;
  //        double Kp, Ki, Kd;

private:
  double lastVE, lastVEI;
  double lastTE, lastTEI;

  double pkp, pki, pkd;
  double tkp, tki, tkd;
  int state;
};

#endif /* _GOTOGOAL_H_ */
