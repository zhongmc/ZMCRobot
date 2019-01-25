
#include "SlidingMode.h"

SlidingMode::SlidingMode()
{
}

void SlidingMode::reset()
{
}

void SlidingMode::execute(Robot *robot, Input *input, Output *output, double dt)
{

  double sensor_gains[] = {1, 1, 0.5, 1, 1};
  IRSensor **irSensors = robot->getIRSensors();
  double uao_x = 0, uao_y = 0;
  for (int i = 0; i < 5; i++)
  {
    uao_x = uao_x + (irSensors[i]->xw - robot->x) * sensor_gains[i];
    uao_y = uao_y + (irSensors[i]->yw - robot->y) * sensor_gains[i];
  }

  u_ao.x = uao_x;
  u_ao.y = uao_y;

  u_gtg.x = input->x_g - robot->x;
  u_gtg.y = input->y_g - robot->y;

  //get the left wall
  int idx = 0;
  for (int i = 1; i < 3; i++)
  {
    if (irSensors[i]->distance >= irSensors[idx]->distance)
      idx = i;
  }

  switch (idx)
  {
  case 0:
    u_fw_l.x = irSensors[2]->xw - irSensors[1]->xw;
    u_fw_l.y = irSensors[2]->yw - irSensors[1]->yw;
    break;
  case 1:
    u_fw_l.x = irSensors[2]->xw - irSensors[0]->xw;
    u_fw_l.y = irSensors[2]->yw - irSensors[0]->yw;

    break;
  case 2:
    u_fw_l.x = irSensors[1]->xw - irSensors[0]->xw;
    u_fw_l.y = irSensors[1]->yw - irSensors[0]->yw;

    break;
  }

  //

  //  %     u_fw_t = p_2-p_1;
  //  %     theta_fw = atan2(u_fw_t(2),u_fw_t(1));
  //  %     obj.u_fw = [x+cos(theta_fw); y+sin(theta_fw)];

  //get the right wall

  idx = 2;
  for (int i = 3; i < 5; i++)
  {
    if (irSensors[i]->distance > irSensors[idx]->distance)
      idx = i;
  }

  switch (idx)
  {
  case 2:
    u_fw_r.x = irSensors[3]->xw - irSensors[4]->xw;
    u_fw_r.y = irSensors[3]->yw - irSensors[4]->yw;
    break;
  case 3:
    u_fw_r.x = irSensors[2]->xw - irSensors[4]->xw;
    u_fw_r.y = irSensors[2]->yw - irSensors[4]->yw;

    break;
  case 4:
    u_fw_r.x = irSensors[2]->xw - irSensors[3]->xw;
    u_fw_r.y = irSensors[2]->yw - irSensors[3]->yw;

    break;
  }

  leftObstacle = false;
  rightObstacle = false;

  double maxDistance = irSensors[0]->getMaxDistance();

  double obsL = min(irSensors[0]->distance, irSensors[1]->distance);
  double obsR = min(irSensors[3]->distance, irSensors[4]->distance);

  if (obsL < obsR)
  {
    if (obsL < maxDistance - 0.05)
      leftObstacle = true;
  }
  else
  {
    if (obsR < maxDistance - 0.05)
      rightObstacle = true;
  }

  // [u_gtg, u_ao][sigma_l] = [u_fw_l]
  //[u_gtg, u_ao][sigma_r] = [u_fw_r] when sigma > 0 pressents that the u_fw is between the gtg and ao vector
  double a1, a2;
  if (u_ao.y == 0 || u_gtg.x == 0)
  {
    Serial.println("Div by zero 1!");
    return;
  }
  a1 = -u_ao.x / u_ao.y;
  a2 = -u_gtg.y / u_gtg.x;

  double fv1, fv2;

  fv1 = (u_gtg.x + a1 * u_gtg.y);
  fv2 = (u_ao.y + a2 * u_ao.x);
  if (fv1 == 0 || fv2 == 0)
  {
    Serial.println("Div by zero 2!");
    return;
  }
  sigma_l.x = (u_fw_l.x + a1 * u_fw_l.y) / fv1;
  sigma_l.y = (u_fw_l.y + a2 * u_fw_l.x) / fv2;

  //fv1 = (u_gtg.x + a1*u_gtg.y);
  //fv2 =  (u_ao.y + a2*u_ao.x);
  //if( fv1 == 0 || fv2 == 0 )
  //{
  //  Serial.println("Div by zero 3!");
  //  return;
  //}

  sigma_r.x = (u_fw_r.x + a1 * u_fw_r.y) / fv1;
  sigma_r.y = (u_fw_r.y + a2 * u_fw_r.x) / fv2;

  slideLeft = sigma_l.x > 0 && sigma_l.y > 0;
  slideRight = sigma_r.x > 0 && sigma_r.y > 0;

#ifdef _DEBUG_
  Serial.print("Slide:sl=");
  Serial.print(slideLeft);
  Serial.print(",sr=");
  Serial.print(slideRight);
  Serial.print(",lo=");
  Serial.print(leftObstacle);
  Serial.print(",ro=");
  Serial.println(rightObstacle);
#endif

  output->v = 0;
  output->w = 0;
}

bool SlidingMode::slidingLeft()
{
  // if( !leftObstacle && !rightObstacle)
  //   return false;

  // if( slideLeft && slideRight )
  //       return  leftObstacle; //(sigma_l.x > 0 && sigma_l.y > 0 );
  // else
  //     return slideLeft;

  return leftObstacle && slideLeft;
}

bool SlidingMode::slidingRight()
{
  //  if( !leftObstacle && !rightObstacle)
  //   return false;

  // if( slideLeft && slideRight )
  //       return  rightObstacle; //(sigma_l.x > 0 && sigma_l.y > 0 );
  // else
  //     return slideRight;

  return rightObstacle && slideRight;
}

bool SlidingMode::quitSlidingLeft()
{

  if (!rightObstacle && !leftObstacle) //no obstacles quit
    return true;

  return !slideLeft; //!(sigma_l.x > 0 && sigma_l.y > 0 );
}

bool SlidingMode::quitSlidingRight()
{
  if (!rightObstacle && !leftObstacle) //no obstacles quit
    return true;
  return !slideRight; //!(sigma_r.x > 0 && sigma_r.y > 0 );
}
