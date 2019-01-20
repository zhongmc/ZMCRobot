#include "RearDriveRobot.h"

RearDriveRobot::RearDriveRobot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  init(0.03181, 0.165, 330, 360, 80, 150, GP2Y0A41); //0.065/2 0.15

  mPIDSettings.kp = 20; //25;  //20 0.5 2
  mPIDSettings.ki = 0.5;
  mPIDSettings.kd = 2.0;
}

Vel RearDriveRobot::ensure_w(double v, double w)
{
  Vel vel;

  if (abs(v) > 0)
  {
    double v_lim, w_lim;
    v_lim = abs(v) / (1 + abs(w));
    if (v_lim > max_v)
      v_lim = max_v;

    if (v_lim < min_v)
      v_lim = min_v;

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
    else if (vel_rl_min < 0) // min_vel)
    {
      vel.vel_r = vel_d.vel_r - vel_rl_min; //(min_vel - vel_rl_min);
      vel.vel_l = vel_d.vel_l - vel_rl_min; //(min_vel - vel_rl_min);
    }
    else
    {
      vel.vel_r = vel_d.vel_r;
      vel.vel_l = vel_d.vel_l;
    }

    if (v < 0)
    {
      vel.vel_l = -vel.vel_l;
      vel.vel_r = -vel.vel_r;
    }
  }
  else
  {
    if (abs(w) < min_w)
    {
      if (w < 0)
      {
        w = -1 * (min_w + 0.01);
      }
      else
        w = (min_w + 0.01);
    }
    vel = uni_to_diff(v, w);
  }
  return vel;
}

double RearDriveRobot::vel_l_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);
  if (nvel < min_vel)
    return 0;
  //  nvel = min_vel;

  if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 0.5729 * nvel * nvel - 5.1735 * nvel + 86.516;
  // double retVal = 9.1631 * nvel + 27.898; //6.393 * nvel + 13.952;

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
    return 0; //nvel = min_vel;

  if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 0.5649 * nvel * nvel - 4.3156 * nvel + 80.706;
  // double retVal = 9.1631 * nvel + 27.898; // 6.2798 * nvel + 18.787;
  //  y = 0.5649x2 - 4.3156x + 80.706

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double RearDriveRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 70) //14
    return 0;

  double ticks = dt * (-0.024 * npwm * npwm + 12.097 * npwm - 426.23);
  //5.7132x - 180.16
  //  double ticks = dt * (5.7132 * npwm - 180.16);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double RearDriveRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 70)
    return 0;

  double ticks = dt * (-0.0218 * npwm * npwm + 11.634 * npwm - 358.83);
  //5.7049x - 131.73
  //  double ticks = dt * (5.7049 * npwm - 131.73);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
