#ifndef _BALANCE_CONTROLLER_H_
#define _BALANCE_CONTROLLER_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class BalanceController :public Controller{
    public:
        BalanceController();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);


 };


#endif /* _BALANCE_CONTROLLER_H_ */





