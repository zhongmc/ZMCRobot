#ifndef _IRRECEIVER_H_
#define _IRRECEIVER_H_

#include <Arduino.h>
#include "ZMCRobot.h"


typedef struct {
    int addr;
    int code;
} IRCode;


class IRReceiver
{
public:
  IRReceiver();
  IRReceiver(byte irpin);

    int readIRCode(IRCode &code);


private:
    long prevMillis, curMillis;
    int pulse_width_l, pulse_width_h;

    byte irpin;
    int ir_code, adrL_code, adrH_code;   


    void readCode();
    int logicValue();
};

#endif /* _IRRECEIVER_H_ */
