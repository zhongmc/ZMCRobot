
#include "Supervisor.h"
#include "ZMCRobot.h"

#define MAX_IRSENSOR_DIS 0.3

Supervisor::Supervisor()
{
  d_fw = 0.20; //distance to follow wall
  d_stop = 0.02;
  d_at_obs = 0.25;
  d_unsafe = 0.1;
  d_prog = 100;

  m_input.x_g = 1;
  m_input.y_g = 0;
  m_input.v = 0.3;
  m_FollowWall.d_fw = 0.20;
  m_FollowWall.dir = 0;

  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters

  // robot.setVel2PwmParam(0,9.59,18.73);
  // robot.setIRSensorType(GP2Y0A21);
  // robot.setHaveIrSensor(0, true);
  // robot.setHaveIrSensor(1, true);
  // robot.setHaveIrSensor(2, false);
  // robot.setHaveIrSensor(3, true);
  // robot.setHaveIrSensor(4, true);

  // m_dkp = 10, m_dki = 0.20, m_dkd = 0.1; // direction
  m_pkp = 5, m_pki = 0.5, m_pkd = 0.1; // position
  m_tkp = 5, m_tki = 10, m_tkd = 0.1;  // theta

  mSimulateMode = false;
  mIgnoreObstacle = false;

  danger = false;
}

void Supervisor::setHaveIRSensor(int idx, bool val)
{
  robot.setHaveIrSensor(idx, val);
}

void Supervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 5)
  {
    d_at_obs = settings.atObstacle;
    d_unsafe = settings.unsafe;
    d_fw = settings.dfw;
    m_input.v = settings.velocity;
    robot.updateSettings(settings);
    m_FollowWall.d_fw = settings.dfw;
  }

  if (settings.sType == 0 || settings.sType == 1 || settings.sType == 2)
  {
    robot.updatePID(settings);
    m_GoToGoal.updateSettings(settings);
    m_AvoidObstacle.updateSettings(settings);
    m_FollowWall.updateSettings(settings);

    // m_dkp = settings.kp;
    // m_dki = settings.ki;
    // m_dkd = settings.kd;
  }
  else if (settings.sType == 3)
  {
    m_pkp = settings.kp;
    m_pki = settings.ki;
    m_pkd = settings.kd;
    m_GoToGoal.setPID(3, m_pkp, m_pki, m_pkd); // updateSettings(settings);
  }
  else if (settings.sType == 4)
  {
    m_tkp = settings.kp;
    m_tki = settings.ki;
    m_tkd = settings.kd;
    m_GoToGoal.setPID(4, m_tkp, m_tki, m_tkd);
  }
}

SETTINGS Supervisor::getSettings(byte settingsType)
{
  SETTINGS settings;

  if (settingsType == 0 || settingsType == 5 || settingsType == 1 || settingsType == 2)
  {
    settings.sType = settingsType;

    settings.atObstacle = d_at_obs;
    settings.unsafe = d_unsafe;
    settings.dfw = d_fw;
    settings.velocity = m_input.v;
    settings.max_rpm = robot.max_rpm;
    settings.min_rpm = robot.min_rpm;

    settings.radius = robot.wheel_radius;
    settings.length = robot.wheel_base_length;
    SETTINGS pidSettings = robot.getPIDParams();
    settings.kp = pidSettings.kp;
    settings.ki = pidSettings.ki;
    settings.kd = pidSettings.kd;
  }

  else if (settingsType == 3) //position control pid
  {
    settings.sType = 3;
    settings.kp = m_pkp;
    settings.ki = m_pki;
    settings.kd = m_pkd;
  }
  else if (settingsType == 4) //theta control pid
  {
    settings.sType = 4;
    settings.kp = m_tkp;
    settings.ki = m_tki;
    settings.kd = m_tkd;
  }

  // m_GoToGoal.getSettings(&settings);

  return settings;
}

void Supervisor::init()
{
  SETTINGS settings = robot.getPIDParams();
  settings.sType = 1;
  m_GoToGoal.updateSettings(settings);
  m_AvoidObstacle.updateSettings(settings);
  m_FollowWall.updateSettings(settings);

  m_GoToGoal.setPID(3, m_pkp, m_pki, m_pkd);
  m_GoToGoal.setPID(4, m_tkp, m_tki, m_tkd);
}

void Supervisor::setGoal(double x, double y, int theta)
{
  m_Goal.x = x;
  m_Goal.y = y;
  m_input.x_g = x;
  m_input.y_g = y;
  m_input.theta = theta;

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

  // robot.x = 0;
  // robot.y = 0;
  // robot.theta = 0;

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

void Supervisor::setObstacleDistance(double dis[5])
{
  robot.setObstacleDistance(dis);
}

void Supervisor::setRobotPosition(double x, double y, double theta)
{
  robot.x = x;
  robot.y = y;
  robot.theta = theta;
}

void Supervisor::execute(long left_ticks, long right_ticks, double dt)
{

  long startTime = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
    robot.updateState(left_ticks, right_ticks, dt);

  if (m_state == S_STOP && at_goal)
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

  if (m_currentController == NULL) //unsafe stoped
    return;

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

  // float v1, v2;
  // v1 = m_output.v;
  // v2 = m_output.v;

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

  // if (m_distanceToGoal < 0.5)
  // {
  //   v2 = m_distanceToGoal * v1;
  // }

  float w = max(min(m_output.w, robot.max_w), -robot.max_w);
  float v = m_output.v; // min(v1, v2);

  if (v != 0 && v < robot.min_v)
    v = 1.01 * robot.min_v;

  m_output.v = v;
  m_output.w = w;

  mVel = robot.ensure_w(v, w);

  PWM_OUT pwm;

  pwm.pwm_l = (int)robot.vel_l_to_pwm(mVel.vel_l);
  pwm.pwm_r = (int)robot.vel_r_to_pwm(mVel.vel_r);

  // Serial.print(mVel.vel_l);
  // Serial.print(",");
  // Serial.println(mVel.vel_r);

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

  //send robot position
  Serial.print("RP");
  Serial.print((int)(10000 * robot.x));
  Serial.print(",");
  Serial.print((int)(10000 * robot.y));
  Serial.print(",");
  Serial.print((int)(10000 * robot.theta));
  Serial.print(",");
  Serial.println(robot.velocity);

  //    Serial.print( nowMicros - timer);
  //    Serial.print(",");
  //    Serial.print(pwm.pwm_l);
  //    Serial.print(",");
  //    Serial.println(pwm.pwm_r);

  execTime = micros() - startTime;
}

/**
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
*/
/**
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

*/

void Supervisor::executeAvoidAndGotoGoal(double dt)
{
  if (unsafe)
  {
    if (m_state != S_STOP)
      Serial.println("USF, stop");
    m_state = S_STOP;
    StopMotor();
    m_currentController = NULL;
    return;
  }

  if (m_state == S_STOP && !unsafe) //recover from stop
  {
    m_state = S_GTG; //gotoGoal;
    m_currentController = &m_GoToGoal;
    m_GoToGoal.reset();
    return;
  }

  if (m_currentController != &m_GoToGoal && m_currentController != &m_FollowWall)
  {
    m_currentController = &m_GoToGoal;
    m_GoToGoal.reset();
    m_state = S_GTG;
    return;
  }

  if (m_state == S_GTG) //goto goal state
  {
    if (at_obstacle)
    {
      changeToFollowWall();
      return;
    }
  }
  else //follow wall
  {
    if (!progress_made)
    {
      if (noObstacle)
      {
        m_state = S_GTG;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        Serial.println("Chg to GTG while no obs!");
      }
      return;
    }

    m_SlidingMode.execute(&robot, &m_input, &m_output, 0.02);
    if (m_FollowWall.dir == 0 && m_SlidingMode.quitSlidingLeft())
    {
      m_state = S_GTG;
      m_currentController = &m_GoToGoal;
      m_GoToGoal.reset();
      Serial.println("Chg to GTG from FW L");
    }
    else if (m_FollowWall.dir == 1 && m_SlidingMode.quitSlidingRight())
    {
      m_state = S_GTG;
      m_currentController = &m_GoToGoal;
      m_GoToGoal.reset();
      Serial.println("Chg to GTG from FW R");
    }
  }
}

void Supervisor::changeToFollowWall()
{
  m_SlidingMode.execute(&robot, &m_input, &m_output, 0.02);

  if (m_SlidingMode.slidingLeft())
  {
    m_FollowWall.dir = 0; //left
    m_currentController = &m_FollowWall;
    m_state = S_FW; //followWall;
    m_FollowWall.reset();
    set_progress_point();
    Serial.println("FLW-L");
  }
  else if (m_SlidingMode.slidingRight())
  {
    m_FollowWall.dir = 1; //right
    m_currentController = &m_FollowWall;
    m_state = S_FW; //followWall;
    m_FollowWall.reset();
    set_progress_point();
    Serial.println("FLW-R");
  }
  else
  {
    Serial.println("FLW failed!");

    m_FollowWall.dir = getOstacleDir() - 1;
    if (m_FollowWall.dir == 0)
      Serial.println("FLW-L");
    else
      Serial.println("FLW-R");
    // m_FollowWall.dir = 0; //left
    m_currentController = &m_FollowWall;
    m_state = S_FW; //followWall;
    m_FollowWall.reset();
    set_progress_point();
  }
}

int Supervisor::getOstacleDir()
{

  IRSensor **irSensors = robot.getIRSensors();

  double maxDis = irSensors[0]->getMaxDistance() - 0.01;

  int l = 0;
  for (int i = 0; i < 3; i++)
  {
    if (irSensors[i]->distance < maxDis)
    {
      l++;
    }
  }

  int r = 0;
  for (int i = 2; i < 5; i++)
  {
    if (irSensors[i]->distance < maxDis)
    {
      r++;
    }
  }

  if (l == 0 && r == 0)
    return 0;

  if (l >= r)
    return 1;
  else
    return 2;
}

void Supervisor::set_progress_point()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));
  d_prog = d;
  // Serial.print("PP:");
  // Serial.print(robot.x);
  // Serial.print(",");
  // Serial.print(robot.y);
  // Serial.print("; d:");
  // Serial.println(d_prog);
}

void Supervisor::check_states()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));

  noObstacle = true;

  m_distanceToGoal = d;

  if (d < (d_prog - 0.3))
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
  for (int i = 1; i < 4; i++)
  {
    if (irSensors[i]->distance < d_at_obs)
      at_obstacle = true;
    if (irSensors[i]->distance < irSensors[i]->getMaxDistance() - 0.01) //  MAX_IRSENSOR_DIS)
      noObstacle = false;
  }

  if (noObstacle)
  {
    if (irSensors[0]->distance < irSensors[0]->getMaxDistance() - 0.01)
      noObstacle = false;
    else if (irSensors[4]->distance < irSensors[4]->getMaxDistance() - 0.01)
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

void Supervisor::readIRDistances(double dis[5])
{
  robot.readIRSensors();
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}