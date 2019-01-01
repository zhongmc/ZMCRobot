
#include "AvoidObstacle.h"



AvoidObstacle::AvoidObstacle()
{
  Kp = 5;
  Ki = 0.01;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
}


void AvoidObstacle::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;

}


void AvoidObstacle::execute(Robot *robot, Input *input, Output* output, double dt)
{

  double sensor_gains[] = {1, 1, 0.5, 1, 1};
  IRSensor **irSensors = robot->getIRSensors();
  double uao_x = 0, uao_y = 0;
  double  e_k, e_I, e_D, w, theta_ao;

  for( int i=0; i<5; i++)
  {
    uao_x = uao_x + (irSensors[i]->xw - robot->x) * sensor_gains[i];
    uao_y = uao_y + (irSensors[i]->yw - robot->y) * sensor_gains[i];
  }
   theta_ao = atan2(uao_y, uao_x );

  double maxDistance = irSensors[0]->getMaxDistance();

  //avoid vertical to obstacle
  if( (irSensors[1]->distance < maxDistance && irSensors[3]->distance < maxDistance ) || irSensors[2]->distance < maxDistance )
  {
    double theta = atan2(irSensors[2]->yw - robot->y, irSensors[2]->xw - robot->x);
    double dif = theta - theta_ao;
    
    if( abs(dif) < 0.2)
    {
      if( dif < 0 )
      {
          theta_ao = atan2(irSensors[1]->yw - robot->y, irSensors[1]->xw - robot->x);
      }
      else
      {
          theta_ao = atan2(irSensors[3]->yw - robot->y, irSensors[3]->xw - robot->x);
        
      }
    }
  }

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







