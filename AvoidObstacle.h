#ifndef _AVOIDOBSTACLE_H_
#define _AVOIDOBSTACLE_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class AvoidObstacle :public Controller{
    public:
        AvoidObstacle();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);

   };


#endif /* _AVOIDOBSTACLE_H_ */





