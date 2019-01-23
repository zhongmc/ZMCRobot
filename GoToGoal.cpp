
#include "GoToGoal.h"

GoToGoal::GoToGoal()
{
  Kp = 5;
  Ki = 0.06;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
  lastVE = 0;
  lastVEI = 0;
}

void GoToGoal::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
  lastVE = 0;
  lastVEI = 0;
}

void GoToGoal::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double u_x, u_y, e, e_I, e_D, w, theta_g;

  u_x = input->x_g - robot->x;
  u_y = input->y_g - robot->y;
  theta_g = atan2(u_y, u_x);

  e = theta_g - robot->theta;
  e = atan2(sin(e), cos(e));
  e_I = lastErrorIntegration + e * dt;
  e_D = (e - lastError) / dt;
  w = Kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;

#ifdef _DEBUG_

  Serial.print(",");
  Serial.print(e);
  Serial.print(",");
  Serial.print(w);

#endif

  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));
  output->v = input->v;

  if (d < 0.02) //slow down
  {
    output->v = 0;
  }
  else if (d < 1) //target controll
  {
    double vei = lastVEI + d * dt;
    double ved = (d - lastVE) / dt;
    output->v = Kp * d + Kd * ved; // 不能有超调
    lastVEI = vei;
    lastVE = d;
  }

  output->w = w;
  lastError = e;

  // Serial.print(theta_g);
  // Serial.print(",");
  // Serial.print(robot->theta);
  // Serial.print(",");
  // Serial.print(e);

  // Serial.print(",");
  // Serial.println(w);
}
