
#include "Supervisor.h"
#include "ZMCRobot.h"

#define MAX_IRSENSOR_DIS 0.3

Supervisor::Supervisor()
{
  d_fw = 0.25; //distance to follow wall
  d_stop = 0.02;
  d_at_obs = 0.18;
  d_unsafe = 0.05;
  d_prog = 100;

  m_input.x_g = 1;
  m_input.y_g = 0;
  m_input.v = 0.4;
  m_FollowWall.d_fw = 0.25;
  m_FollowWall.dir = 0;

  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters

  // robot.setVel2PwmParam(0,9.59,18.73);
  robot.setIRSensorType(GP2Y0A21);

  robot.setHaveIrSensor(0, false);
  robot.setHaveIrSensor(2, false);
  robot.setHaveIrSensor(3, false);
  robot.setHaveIrSensor(4, false);

  mSimulateMode = false;
  mIgnoreObstacle = false;

  danger = false;
}

void Supervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 4)
  {
    d_at_obs = settings.atObstacle;
    d_unsafe = settings.unsafe;
    d_fw = settings.dfw;
    m_input.v = settings.velocity;
    robot.updateSettings(settings);
    m_FollowWall.d_fw = settings.dfw;
  }

  if (settings.sType == 0 || settings.sType == 1)
  {
    m_GoToGoal.updateSettings(settings);
    m_AvoidObstacle.updateSettings(settings);
    m_FollowWall.updateSettings(settings);
  }
}

SETTINGS Supervisor::getSettings(byte settingsType)
{
  SETTINGS settings;

  settings.atObstacle = d_at_obs;
  settings.unsafe = d_unsafe;
  settings.dfw = d_fw;
  settings.velocity = m_input.v;
  settings.max_rpm = robot.max_rpm;
  settings.min_rpm = robot.min_rpm;
  m_GoToGoal.getSettings(&settings);

  return settings;
}

void Supervisor::init()
{
  SETTINGS settings = robot.getPIDParams();
  m_GoToGoal.updateSettings(settings);
  m_AvoidObstacle.updateSettings(settings);
  m_FollowWall.updateSettings(settings);
}

void Supervisor::setGoal(double x, double y, int theta)
{
  m_Goal.x = x;
  m_Goal.y = y;
  m_input.x_g = x;
  m_input.y_g = y;
  //  robot.theta = 2*PI*theta/360;
}

void Supervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;

  d_prog = 20;
  m_GoToGoal.reset();
  m_AvoidObstacle.reset();
  m_FollowWall.reset();

  m_FollowWall.dir = 0; //left

  m_state = S_GTG; //gotoGoal;
  m_currentController = &m_GoToGoal;

  progress_made = false;
  at_goal = false;
  at_obstacle = false;
  unsafe = false;
  danger = false;
}

void Supervisor::reset(long leftTicks, long rightTicks)
{
  d_prog = 20;
  m_GoToGoal.reset();
  m_AvoidObstacle.reset();
  m_FollowWall.reset();

  m_FollowWall.dir = 0; //left

  m_state = S_GTG; //gotoGoal;
  m_currentController = &m_GoToGoal;

  progress_made = false;
  at_goal = false;
  at_obstacle = false;
  unsafe = false;
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

void Supervisor::execute(long left_ticks, long right_ticks, double dt)
{

  uint32_t timer = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
    robot.updateState(left_ticks, right_ticks, dt);


  if( m_state == S_STOP && at_goal )
    return;

  check_states();

  if (at_goal)
  {
    if (m_state != S_STOP)
      Serial.println("At Goal!");
    //    else
    //      Serial.println("");

    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }
  else if (!mSimulateMode && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }
  /////////////////////////////////////////////////////////
  executeAvoidAndGotoGoal(dt);

  m_output.v = 0;
  m_output.w = 0;

  if (m_currentController != NULL)
  {
    //       Serial.println("exec Controller...");grl

    m_currentController->execute(&robot, &m_input, &m_output, dt);
  }

  double obsDis;

  obsDis = robot.getObstacleDistance();
  //  if ( noObstacle ) //obsDis > 0.29 ) //not within obstacle
  //     obsDis = m_distanceToGoal;

  float v1, v2;
  v1 = m_output.v;
  v2 = m_output.v;

  // if (abs(m_output.w) < 5)
  //   v2 = m_output.v / (1 + W_SPEED_DOWN_SCALE * abs(m_output.w) / 5); //W_SPEED_DOWN_SCALE 1
  // else
  //   v2 = m_output.v / (1 + W_SPEED_DOWN_SCALE * abs(m_output.w)); //W_SPEED_DOWN_SCALE 1

  // if (obsDis < MAX_IRSENSOR_DIS) //too close to obstacle, slow down....
  // {
  //   v1 = m_output.v * log10(DIS_SPEED_DOWN_SCALE * obsDis + 1); //DIS_SPEED_DOWN_SCALE 10
  // }
  // else if (m_distanceToGoal < SPEED_DOWN_DIS) // close to goal, slow down
  // {
  //   v1 = m_output.v * log10(DIS_SPEED_DOWN_SCALE * m_distanceToGoal + 1); //DIS_SPEED_DOWN_SCALE 10
  // }

  if (m_distanceToGoal < 0.5)
  {
    v2 = m_distanceToGoal * v1;
  }

  float w = max(min(m_output.w, robot.max_w), -robot.max_w);
  float v = min(v1, v2);

  if (v != 0 && v < robot.min_v)
    v = 1.01 * robot.min_v;

  m_output.v = v;
  m_output.w = w;

  mVel = robot.ensure_w(v, w);

  PWM_OUT pwm;

  pwm.pwm_l = (int)robot.vel_l_to_pwm(mVel.vel_l);
  pwm.pwm_r = (int)robot.vel_r_to_pwm(mVel.vel_r);

  // Serial.print(",");

/*
  Serial.print(m_output.v);
  Serial.print(",");

  Serial.print(v);
  Serial.print(",");

  Serial.print(vel.vel_l);
  Serial.print(",");
  Serial.print(vel.vel_r);

  Serial.print(pwm.pwm_l);
  Serial.print(",");
  Serial.println(pwm.pwm_r);

*/
#ifdef _DEBUG_
  Serial.print(robot.x);
  Serial.print(",");
  Serial.print(robot.y);

  Serial.print(",");
  Serial.print(robot.theta);

  Serial.print(",");
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

  // uint32_t nowMicros = micros();

  execTime = micros() - timer;

  //    Serial.print( nowMicros - timer);
  //    Serial.print(",");
  //    Serial.print(pwm.pwm_l);
  //    Serial.print(",");
  //    Serial.println(pwm.pwm_r);
}

void Supervisor::executeFollowWall(double dt)
{

  if (m_state == S_STOP && !unsafe) //recover from stop
  {
    m_state = S_GTG; //gotoGoal;
    m_currentController = &m_GoToGoal;
    m_GoToGoal.reset();
  }

  if (unsafe)
  {
    if (m_state != S_AVO)
      m_AvoidObstacle.reset();
    m_state = S_AVO; // avoidObstacle;
    m_currentController = &m_AvoidObstacle;
    Serial.println("unsafe , change to avoidObstacle");
  }
  else
  {
    //      Serial.println("exec sliding");

    if (at_obstacle || m_state == S_FW)
      m_SlidingMode.execute(&robot, &m_input, &m_output, dt);

    if (m_state == S_GTG)
    {
      if (at_obstacle && m_SlidingMode.slidingLeft())
      {
        m_FollowWall.dir = 0; //left
        m_currentController = &m_FollowWall;
        m_state = S_FW; //followWall;
        m_FollowWall.reset();
        set_progress_point();
        Serial.println("Change to follow wall left state ");
      }
      else if (at_obstacle && m_SlidingMode.slidingRight())
      {
        m_FollowWall.dir = 1; //right
        m_currentController = &m_FollowWall;
        m_state = S_FW; //followWall;
        m_FollowWall.reset();
        set_progress_point();
        Serial.println("Change to follow wall right state ");
      }
      else if (at_obstacle)
      {
        m_currentController = &m_AvoidObstacle;
        m_state = S_AVO; //avoidObstacle;
        m_AvoidObstacle.reset();
        Serial.println("Change to Avoid obstacle ");
      }
    }
    else if (m_state == S_FW) //followWall )
    {
      if (progress_made)
      {
        if (m_FollowWall.dir == 0 && !m_SlidingMode.quitSlidingLeft())
        {
          m_state = S_GTG; //gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to go to goal state ");
        }
        else if (m_FollowWall.dir == 1 && m_SlidingMode.quitSlidingRight())
        {
          m_state = S_GTG; // gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to go to goal state ");
        }
      }
    }
    else if (m_state == S_AVO) // avoidObstacle)
    {
      if (!at_obstacle)
      {
        if (m_SlidingMode.slidingLeft())
        {
          m_FollowWall.dir = 0; //left
          m_currentController = &m_FollowWall;
          m_state = S_FW; //followWall;
          m_FollowWall.reset();
          set_progress_point();
          Serial.println("Change to follow wall left  state (from avo) ");
        }
        if (m_SlidingMode.slidingRight())
        {
          m_FollowWall.dir = 1; //right
          m_currentController = &m_FollowWall;
          m_state = S_FW; //followWall;
          m_FollowWall.reset();
          set_progress_point();
          Serial.println("Change to go to follow wall right  state (from avo) ");
        }
        else
        {
          m_state = S_GTG; //gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to Go to goal  state (from avo) ");
        }
      }
    }
  }
}

void Supervisor::executeAvoidAndGotoGoal(double dt)
{

  if (m_state == S_STOP && !unsafe) //recover from stop
  {
    m_state = S_GTG; //gotoGoal;
    m_currentController = &m_GoToGoal;
    m_GoToGoal.reset();
  }

  if (unsafe)
  {
    if (m_state != S_AVO)
      m_AvoidObstacle.reset();
    m_state = S_AVO; // avoidObstacle;
    m_currentController = &m_AvoidObstacle;
    Serial.println("unsafe , change to avoidObstacle");
  }
  else
  {
    //      Serial.println("exec sliding");

    if (at_obstacle || m_state == S_FW)
      m_SlidingMode.execute(&robot, &m_input, &m_output, dt);

    if (m_state == S_GTG)
    {
      if (at_obstacle && m_SlidingMode.slidingLeft())
      {
        m_FollowWall.dir = 0; //left
        m_currentController = &m_FollowWall;
        m_state = S_FW; //followWall;
        m_FollowWall.reset();
        set_progress_point();
        Serial.println("Change to follow wall left state ");
      }
      else if (at_obstacle && m_SlidingMode.slidingRight())
      {
        m_FollowWall.dir = 1; //right
        m_currentController = &m_FollowWall;
        m_state = S_FW; //followWall;
        m_FollowWall.reset();
        set_progress_point();
        Serial.println("Change to follow wall right state ");
      }
      else if (at_obstacle)
      {
        m_currentController = &m_AvoidObstacle;
        m_state = S_AVO; //avoidObstacle;
        m_AvoidObstacle.reset();
        Serial.println("Change to Avoid obstacle ");
      }
    }
    else if (m_state == S_FW) //followWall )
    {
      if (progress_made)
      {
        if (m_FollowWall.dir == 0 && !m_SlidingMode.quitSlidingLeft())
        {
          m_state = S_GTG; //gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to go to goal state ");
        }
        else if (m_FollowWall.dir == 1 && m_SlidingMode.quitSlidingRight())
        {
          m_state = S_GTG; // gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to go to goal state ");
        }
      }
      if (noObstacle)
      {
        m_state = S_GTG; // gotoGoal;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        Serial.println("Change to go to goal state ");
      }
    }
    else if (m_state == S_AVO) // avoidObstacle)
    {
      if (!at_obstacle)
      {
        if (m_SlidingMode.slidingLeft())
        {
          m_FollowWall.dir = 0; //left
          m_currentController = &m_FollowWall;
          m_state = S_FW; //followWall;
          m_FollowWall.reset();
          set_progress_point();
          Serial.println("Change to follow wall left  state (from avo) ");
        }
        if (m_SlidingMode.slidingRight())
        {
          m_FollowWall.dir = 1; //right
          m_currentController = &m_FollowWall;
          m_state = S_FW; //followWall;
          m_FollowWall.reset();
          set_progress_point();
          Serial.println("Change to go to follow wall right  state (from avo) ");
        }
        else
        {
          m_state = S_GTG; //gotoGoal;
          m_currentController = &m_GoToGoal;
          m_GoToGoal.reset();
          Serial.println("Change to Go to goal  state (from avo) ");
        }
      }
    }
  }
}

void Supervisor::set_progress_point()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));
  d_prog = d;
  Serial.print("Set progress Point:");
  Serial.print(robot.x);
  Serial.print(",");
  Serial.print(robot.y);
  Serial.print("; d:");
  Serial.println(d_prog);
}

void Supervisor::check_states()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));

  noObstacle = true;

  m_distanceToGoal = d;

  if (d < (d_prog - 0.1))
    progress_made = true;
  else
    progress_made = false;

  if (d < d_stop)
    at_goal = true;
  else
    at_goal = false;

  at_obstacle = false;
  unsafe = false;

  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    if (irSensors[i]->distance < d_at_obs)
      at_obstacle = true;
    if (irSensors[i]->distance < MAX_IRSENSOR_DIS)
      noObstacle = false;
  }

  if (mIgnoreObstacle)
    at_obstacle = false;

  if (irSensors[1]->distance < d_unsafe || irSensors[2]->distance < d_unsafe || irSensors[3]->distance < d_unsafe)
    unsafe = true;

  danger = false;
  if (irSensors[1]->distance < d_unsafe && irSensors[2]->distance < d_unsafe && irSensors[3]->distance < d_unsafe)
    danger = true;
}

Position Supervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void Supervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}
