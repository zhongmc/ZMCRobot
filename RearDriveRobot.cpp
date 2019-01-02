#include "RearDriveRobot.h"

RearDriveRobot::RearDriveRobot()
{
  //R, L, ticksr, minRpm, maxRpm, GP2Y0A41);
  init(0.065 / 2, 0.15, 330, 35, 220, GP2Y0A41);

  mPIDSettings.kp = 30;
  mPIDSettings.ki = 0.2;
  mPIDSettings.kd = 0.1;
}

Vel RearDriveRobot::ensure_w(double v, double w)
{
  Vel vel;

  if (abs(v) > 0)
  {
    double v_lim, w_lim;
    v_lim = abs(v);
    if (v_lim > this->max_v)
      v_lim = max_v;
    w_lim = abs(w);
    if (w_lim > max_w)
      w_lim = max_w;

    if (w < 0)
      w_lim = -1 * w_lim;

    Vel vel_d = uni_to_diff(v_lim, w_lim);
    double vel_rl_max, vel_rl_min;
    if (vel_d.vel_l > vel_d.vel_r)
    {
      vel_rl_min = vel_d.vel_r;
      vel_rl_max = vel_d.vel_l;
    }
    else
    {
      vel_rl_min = vel_d.vel_l;
      vel_rl_max = vel_d.vel_r;
    }

    if (vel_rl_max > max_vel)
    {
      vel.vel_r = vel_d.vel_r - (vel_rl_max - max_vel);
      vel.vel_l = vel_d.vel_l - (vel_rl_max - max_vel);
    }
    else if (vel_rl_min < min_vel)
    {
      vel.vel_r = vel_d.vel_r + (min_vel - vel_rl_min);
      vel.vel_l = vel_d.vel_l + (min_vel - vel_rl_min);
    }
    else
    {
      vel.vel_r = vel_d.vel_r;
      vel.vel_l = vel_d.vel_l;
    }
  }
  else
  {
    if (w < 0)
    {
      w = -1 * min_w;
    }
    else
      w = min_w;
    vel = uni_to_diff(v, w);
  }
  return vel;
}

double RearDriveRobot::vel_l_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);
  if (nvel < min_vel)
    nvel = min_vel;
  else if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 9.1631 * nvel + 27.898; //6.393 * nvel + 13.952;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double RearDriveRobot::vel_r_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);

  if (nvel < min_vel)
    nvel = min_vel;
  else if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 9.1631 * nvel + 27.898; // 6.2798 * nvel + 18.787;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double RearDriveRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 70)
    return 0;

  double ticks = dt * (-0.0219 * npwm * npwm + 12.578 * npwm - 398.53);
  //5.7049x - 131.73
  //  double ticks = dt * (5.7049 * npwm - 131.73);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double RearDriveRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 70) //14
    return 0;

  double ticks = dt * (-0.0264 * npwm * npwm + 13.824 * npwm - 529.52);
  //5.7132x - 180.16
  //  double ticks = dt * (5.7132 * npwm - 180.16);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
