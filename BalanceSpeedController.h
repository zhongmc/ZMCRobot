#ifndef _SPEED_CONTROLLER_H_
#define _SPEED_CONTROLLER_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class BalanceSpeedController: public Controller{
    public:
        BalanceSpeedController();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);
  

 
 };


#endif /* _SPEED_CONTROLLER_H_ */





