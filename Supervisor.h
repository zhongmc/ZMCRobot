
#ifndef _SUPERVISOR_H_
#define _SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"

#include "GoToGoal.h"
#include "AvoidObstacle.h"
#include "FollowWall.h"
#include "SlidingMode.h"

#define S_STOP 0
#define S_GTG 1
#define S_AVO 2
#define S_FW 3

class Supervisor
{
public:
  Supervisor();
  void execute(long left_ticks, long right_ticks, double dt);

  void executeFollowWall(double dt);
  void executeAvoidAndGotoGoal(double dt);

  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void setGoal(double x, double y, int theta);

  void setHaveIRSensor(int idx, bool val);

  void getRobotInfo()
  {
    Serial.print("GotoGoal robot info, current state:");
    if (m_state == S_STOP)
      Serial.println("Stoped!");
    else if (m_state == S_GTG)
      Serial.println("GotoGoal!");
    else if (m_state == S_AVO)
      Serial.println("AvoidObstacle!");
    else if (m_state == S_FW)
    {
      if (m_FollowWall.dir == 0)
        Serial.println("FollowWall LEFT");
      else
        Serial.println("FollowWall RIGHT");
    }

    Serial.print("input-v:");
    Serial.print(m_input.v);

    Serial.print("; v:");
    Serial.print(m_output.v);
    Serial.print(", w:");
    Serial.print(m_output.w);

    Serial.print(", vel-l:");
    Serial.print(mVel.vel_l);
    Serial.print(", vel-r:");
    Serial.println(mVel.vel_r);
    // long c1, c2;
    // c1 = (long)m_left_ticks;
    // c2 = (long)m_right_ticks;
    Serial.print("c1:");
    Serial.print(m_left_ticks);
    Serial.print(", c2:");
    Serial.println(m_right_ticks);

    Serial.print("exec time:");
    Serial.println(execTime);
    robot.getRobotInfo();
    Serial.print("goto goal ");
    m_GoToGoal.PrintInfo();
    Serial.print("follow wall ");
    m_FollowWall.PrintInfo();
  }

  //the target to go!
  Vector m_Goal;

  void getIRDistances(double dis[5]);
  Position getRobotPosition();

  void init();

  void updateSettings(SETTINGS settings);
  SETTINGS getSettings(byte settingsType);

  bool mSimulateMode;
  bool mIgnoreObstacle;

private:
  void
  set_progress_point();
  void check_states();

  bool progress_made;
  bool at_goal;
  bool at_obstacle;
  bool noObstacle;
  bool unsafe;
  bool danger;

  int m_state;
  double m_right_ticks, m_left_ticks;
  double m_distanceToGoal;

private:
  GoToGoal m_GoToGoal;
  AvoidObstacle m_AvoidObstacle;
  FollowWall m_FollowWall;
  SlidingMode m_SlidingMode;

  // Robot robot;

  RearDriveRobot robot;

  Controller *m_currentController;
  double d_fw; //distance to follow wall

  double d_stop;
  double d_at_obs;
  double d_unsafe;
  double d_prog;

  long execTime;

  Input m_input;
  Output m_output;
  Vel mVel;
};

#endif /* _SUPERVISOR_H_ */
