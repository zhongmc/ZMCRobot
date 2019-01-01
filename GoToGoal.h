#ifndef _GOTOGOAL_H_
#define _GOTOGOAL_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class GoToGoal :public Controller{
    public:
        GoToGoal();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);
//    private:
//        double fuckEk_l;
//        double E_k;
//        double Kp, Ki, Kd;
//     
};


#endif /* _GOTOGOAL_H_ */





