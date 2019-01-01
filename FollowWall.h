
#ifndef _FOLLOWWALL_H_
#define _FOLLOWWALL_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class FollowWall :public Controller{
    public:
        FollowWall();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);
        double d_fw;
        byte dir;  // 0 left 1 right
};


#endif /* _FOLLOWWALL_H_ */





