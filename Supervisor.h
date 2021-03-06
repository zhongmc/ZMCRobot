
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
#define S_FLW 3

class Supervisor
{
public:
  Supervisor();
  void execute(long left_ticks, long right_ticks, double dt);

  // void executeFollowWall(double dt);
  void executeAvoidAndGotoGoal(double dt);

  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void setGoal(double x, double y, int theta, double v);

  void setHaveIRSensor(int idx, byte val);

  void setIRFilter(bool open, float filter);

  void setSimulateMode(bool val);

  void getRobotInfo()
  {
    Serial.print("GTG Robot:");
    if (m_state == S_STOP)
      Serial.println("Stoped!");
    else if (m_state == S_GTG)
      Serial.println("GTG!");
    else if (m_state == S_AVO)
      Serial.println("AVO!");
    else if (m_state == S_FLW)
    {
      if (m_FollowWall.dir == 0)
        Serial.println("FLW L");
      else
        Serial.println("FLW R");
    }

    Serial.print("i-v:");
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

    Serial.print("at ob:");
    Serial.print(d_at_obs);
    Serial.print(", ");
    Serial.print("dfw:");
    Serial.println(d_fw);

    Serial.print("exec time:");
    Serial.println(execTime);
    robot.getRobotInfo();
    Serial.print("GTG CTRL ");
    m_GoToGoal.PrintInfo();
    Serial.print("FLW CTRL ");
    m_FollowWall.PrintInfo();
  }

  //the target to go!
  Vector m_Goal;

  void getIRDistances(double dis[5]);
  void readIRDistances(double dis[5]);

  void setObstacleDistance(double dis[5]);

  Position getRobotPosition();

  void setRobotPosition(double x, double y, double theta);

  void init();

  void updateSettings(SETTINGS settings);
  SETTINGS getSettings(byte settingsType);

  bool mSimulateMode;
  bool mIgnoreObstacle;

private:
  void
  set_progress_point();
  void check_states();
  bool changeToFollowWall();

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

  // double m_dkp, m_dki, m_dkd; // direction
  double m_pkp, m_pki, m_pkd; // position
  double m_tkp, m_tki, m_tkd; // theta

  Vel mVel;
};

#endif /* _SUPERVISOR_H_ */
