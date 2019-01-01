
#include "BalanceController.h"

#include "ZMCRobot.h"


BalanceController::BalanceController()
{
  Kp = 18;     //kp = max out/10 commented
  Ki = 0.0;
  Kd = 1.8;  //kd = kp/70 commented
  lastError = 0;
  lastErrorIntegration = 0;
}


void BalanceController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;

}


void BalanceController::execute(Robot *robot, Input *input, Output* output, double dt)
{
  float  e_k, pidValue, e_I, e_D;
  /* Update PID values */
  e_k = input->targetAngle - robot->angle -  robot->angleOff; //roll;  // pitch;
  // 0, 0.25, 0.4, 0.6, 0.85, 1
  double kp, ne;
  ne = abs(e_k);
//  if( ne < 1 )
//    kp = Kp * 0.25;
//  else if( ne < 2 )
//    kp = Kp * 0.4;
//  else if( ne < 3 )
//    kp = Kp * 0.6;
//  else if( ne < 4 )
//    kp = Kp *0.85;
//  else
    kp = Kp;

 
  e_D = (e_k - lastError )/dt;
  e_I = lastErrorIntegration + e_k * dt;

  if( abs(e_I) > 200 )
    e_I = 0;

  pidValue = kp*e_k - Kd * robot->gyro;
//  pidValue = Kp*e_k + Ki*e_I + Kd * e_D;

  lastError = e_k;
  lastErrorIntegration = e_I;
  
  output->w = pidValue;


}








