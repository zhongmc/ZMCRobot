
#include "BalanceSpeedController.h"



BalanceSpeedController::BalanceSpeedController()
{
  Kp = 8;
  Ki = 0.01;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
}


void BalanceSpeedController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
}


void BalanceSpeedController::execute(Robot *robot, Input *input, Output* output, double dt)
{
  double  e, e_I, e_D, w;

  e = input->v - robot->velocity;
  e_I = lastErrorIntegration + e * dt;
  e_D = (e - lastError )/dt;
  w = Kp * e + Ki * e_I + Kd * e_D;
  
  lastErrorIntegration = e_I;
  if( abs(lastErrorIntegration) > 200 )
    lastErrorIntegration = 0;
#ifdef _DEBUG_
  Serial.print(e);
  Serial.print(",");
  Serial.print(w);
  Serial.print(",");
#endif
  
  output->v = input->v;
  output->w = w;

  lastError = e;
}







