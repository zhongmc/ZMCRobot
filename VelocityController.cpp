
#include "VelocityController.h"

VelocityController::VelocityController()
{
  Kp = 8;
  Ki = 0.01;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
  mTheta = 0;
  mW = 0;
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

  if (mW == 0)
  {
    lastErrorIntegration = 0;
    lastError = 0;
  }
  // if (theta == 0)
  // {
  //   mTheta = curTheta; //to remain this direction??
  //   lastErrorIntegration = 0;
  // }
}

void VelocityController::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double e, e_I, e_D, w;

  //   e = input->theta;
  //   if( e == 0 )
  //   {
  // //      e = robot->theta - mTheta;
  //       e = mTheta - robot->theta;
  //   }

  if (mW != 0) //转弯，开环控制
  {
    output->v = input->v;
    // if (input->v != 0)
    //   output->w = 2 * 10 * input->v * mW;
    // else
    //   output->w = 4 * mW;
    output->w = Kp * mW;
    return;
    // e = mW;
    // p = 10;
  }
  else
  {
    e = mTheta - robot->theta;
    e = atan2(sin(e), cos(e));
  }

  e_I = lastErrorIntegration + e * dt;
  e_D = (e - lastError) / dt;
  w = Kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  if (abs(lastErrorIntegration) > 100)
    lastErrorIntegration = 0;

    // Serial.print(mTheta);
    // Serial.print(",");
    // Serial.print(robot->theta);
    // Serial.print(",");
    // Serial.print(e);
    // Serial.print(",");
    // Serial.println(w);

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
