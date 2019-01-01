#ifndef _AOANDGTG_H_
#define _AOANDGTG_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class AOAndGTG :public Controller{
    public:
        AOAndGTG();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);

    private:
        float alpha;
 };


#endif /* _AOANDGTG_H_ */





