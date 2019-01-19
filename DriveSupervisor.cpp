
#include "DriveSupervisor.h"
#include "ZMCRobot.h"

DriveSupervisor::DriveSupervisor()
{
  d_unsafe = 0.11;

  m_input.x_g = 0;
  m_input.y_g = 0;
  m_input.v = 0.3;
  m_input.theta = 0;

  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters
  //   robot.setVel2PwmParam(0,9.59,18.73);

  robot.setIRSensorType(GP2Y0A21);

  mSimulateMode = false;
  mIgnoreObstacle = false;

  danger = false;
}

void DriveSupervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 4)
  {
    d_unsafe = settings.unsafe;
    m_input.v = settings.velocity;
    robot.updateSettings(settings);
  }

  if (settings.sType == 0 || settings.sType == 1)
    m_Controller.updateSettings(settings);
}

void DriveSupervisor::init()
{
  SETTINGS settings = robot.getPIDParams();
  m_Controller.updateSettings(settings);
}

void DriveSupervisor::setGoal(double v, double theta)
{
  m_input.v = v;
  m_input.theta = theta;

  if (theta == 0 && curTheta != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    curTheta = 0;
    mTheta = robot.theta; //转弯结束，保留当前角度
  }
  else
    curTheta = theta;

  Serial.print("Set drive goal to: ");
  Serial.print(v);
  Serial.print(",");
  Serial.println(theta);
  m_Controller.setGoal(v, theta, mTheta);
}

void DriveSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  m_Controller.setGoal(m_input.v, 0, 0);
  m_Controller.reset();
  mTheta = 0;
  curTheta = 0;
}

void DriveSupervisor::reset(long leftTicks, long rightTicks)
{
  mTheta = 0;
  curTheta = 0;

  m_Controller.reset();
  danger = false;

  if (mSimulateMode)
  {
    m_left_ticks = 0;
    m_right_ticks = 0;
    robot.reset(m_left_ticks, m_right_ticks);
  }
  else
    robot.reset(leftTicks, rightTicks);
}

void DriveSupervisor::execute(long left_ticks, long right_ticks, double dt)
{

  //  uint32_t timer = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
    robot.updateState(left_ticks, right_ticks, dt);

  check_states();

  if (!mSimulateMode && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }

#ifdef _DEBUG_
  Serial.print(robot.x);
  Serial.print(",");
  Serial.print(robot.y);
  Serial.print(",");
  Serial.print(robot.theta);
  Serial.print(",");
#endif

  m_Controller.execute(&robot, &m_input, &m_output, dt);

  // double obsDis = robot.getObstacleDistance();

  // float v = m_output.v;
  // if (abs(m_output.w) < 5)
  //   v = v / (1 + W_SPEED_DOWN_SCALE * abs(m_output.w) / 5); //slow down according to turning w
  // else
  //   v = v / (1 + W_SPEED_DOWN_SCALE * abs(m_output.w)); //slow down according to turning w

  // if (obsDis < 0.10) //danger only allow turning
  // {
  //   v = 0;
  // }
  // else if (obsDis < MAX_IRSENSOR_DIS)
  // {
  //   float v1 = m_output.v * log10(DIS_SPEED_DOWN_SCALE * obsDis + 1); //obsDis*10  slow down according to obstacle
  //   v = min(v, v1);
  // }

  // float w = m_output.w; // max(min(m_output.w, robot.max_w), -robot.max_w);

  // Vel vel;

  v = m_output.v;
  w = m_output.w;

  mVel = robot.ensure_w(v, w);

  PWM_OUT pwm;
  pwm.pwm_l = (int)robot.vel_l_to_pwm(mVel.vel_l);
  pwm.pwm_r = (int)robot.vel_r_to_pwm(mVel.vel_r);

#ifdef _DEBUG_
  Serial.print(v);
  Serial.print(",");
  Serial.print(w);

  Serial.print(",");
  Serial.print(vel.vel_l);
  Serial.print(",");
  Serial.print(vel.vel_r);

  Serial.print(",");
  Serial.print(pwm.pwm_l);
  Serial.print(",");
  Serial.println(pwm.pwm_r);

#endif

  if (mSimulateMode)
  {
    m_left_ticks = m_left_ticks + robot.pwm_to_ticks_l(pwm.pwm_l, dt);
    m_right_ticks = m_right_ticks + robot.pwm_to_ticks_r(pwm.pwm_r, dt);
  }
  else
  {
    MoveLeftMotor(pwm.pwm_l);
    MoveRightMotor(pwm.pwm_r);
  }

  //   uint32_t nowMicros = micros();

  //     Serial.print(",");
  //   Serial.println( nowMicros - timer);
}

extern double ultrasonicDistance;

void DriveSupervisor::check_states()
{

  IRSensor **irSensors = robot.getIRSensors();
  //    for( int i=0; i<5; i++)
  //    {
  //      if( irSensors[i]->distance < d_at_obs )
  //        at_obstacle = true;
  //      if( irSensors[i]->distance < d_unsafe )
  //        unsafe = true;
  //    }

  //if ( irSensors[1]->distance < d_unsafe || irSensors[2]->distance < d_unsafe || irSensors[3]->distance < d_unsafe )

  if (ultrasonicDistance < MAX_ULTRASONIC_DIS)
  {
    if (ultrasonicDistance < 0.05)
      danger = true;
    else
      danger = false;
    return;
  }

  if (irSensors[2]->distance < d_unsafe)
    danger = true;
  else
    danger = false;
}

Position DriveSupervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void DriveSupervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}
