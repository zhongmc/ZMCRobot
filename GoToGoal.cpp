
#include "GoToGoal.h"

GoToGoal::GoToGoal()
{
  Kp = 5;
  Ki = 0.06;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
}

void GoToGoal::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
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

  output->v = input->v;

  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));
  output->v = input->v;

  if (d < 0.1) //slow down
  {
    output->v = 4 * d;
  }
  else
  {
    output->v = input->v / (1 + 10 * abs(theta_g - robot->theta));
  }

  output->w = w;

  lastError = e;
}
