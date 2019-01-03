
#include "VelocityController.h"

VelocityController::VelocityController()
{
  Kp = 8;
  Ki = 0.01;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
  mTheta = 0;
}

void VelocityController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
  mTheta = 0;
}

void VelocityController::setGoal(double v, double theta, double curTheta)
{

  mTheta = curTheta;
  mW = theta;

  // if (theta == 0)
  // {
  //   mTheta = curTheta; //to remain this direction??
  //   lastErrorIntegration = 0;
  // }
}

void VelocityController::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double e, e_I, e_D, w, p;

  p = Kp;
  //   e = input->theta;
  //   if( e == 0 )
  //   {
  // //      e = robot->theta - mTheta;
  //       e = mTheta - robot->theta;
  //   }

  if (mW != 0) //转弯，开环控制
  {
    output->v = input->v;
    output->w = mW;
    lastErrorIntegration = 0;
    // lastError = 0;
    e = mW;
    p = 10;
  }
  else
  {
    e = mTheta - robot->theta;
    e = atan2(sin(e), cos(e));
  }

  e_I = lastErrorIntegration + e * dt;
  e_D = (e - lastError) / dt;
  w = p * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  if (abs(lastErrorIntegration) > 100)
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
