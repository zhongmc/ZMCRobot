
#include "AOAndGTG.h"



AOAndGTG::AOAndGTG()
{
  Kp = 5;
  Ki = 0.01;
  Kd = 0.01;
  alpha = 0.25;
   lastError = 0;
   lastErrorIntegration = 0;

}


void AOAndGTG::reset()
{
   lastError = 0;
   lastErrorIntegration = 0;

}


void AOAndGTG::execute(Robot *robot, Input *input, Output* output, double dt)
{

  double sensor_gains[] = {1, 1, 0.5, 1, 1};
  IRSensor **irSensors = robot->getIRSensors();
  double uao_x = 0, uao_y = 0;
  for( int i=0; i<5; i++)
  {
    uao_x = uao_x + (irSensors[i]->xw - robot->x)* sensor_gains[i];
    uao_y = uao_y + (irSensors[i]->yw - robot->y)* sensor_gains[i];
  }

  
  double ug_x, ug_y, e_k, e_I, e_D, w, theta_ao;

  ug_x = input->x_g - robot->x;
  ug_y = input->y_g - robot->y;

  double ux,uy;
  ux = ug_x*alpha + (1-alpha)*uao_x;
  uy = ug_y*alpha +(1-alpha)*uao_y;

   theta_ao = atan2(uy,ux );
   e_k = theta_ao- robot->theta;
   e_k = atan2(sin(e_k),cos(e_k));

  
  e_I = lastErrorIntegration + e_k * dt;
  e_D = (e_k - lastError )/dt;
  w = Kp * e_k + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;

  output->v = input->v;
  output->w = w;

  lastError = e_k;

}






