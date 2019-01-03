
#ifndef _DRIVE_SUPERVISOR_H_
#define _DRIVE_SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"

#include "VelocityController.h"

#define S_STOP 0
#define S_GTG 1
#define S_AVO 2
#define S_FW 3

class DriveSupervisor
{
  public:
    DriveSupervisor();
    void execute(long left_ticks, long right_ticks, double dt);
    void reset(long leftTicks, long rightTicks);
    void resetRobot();
    void setGoal(double v, double w);

    void getRobotInfo()
    {
        Serial.println("Drive robot info:");
        robot.getRobotInfo();
        m_Controller.PrintInfo();
    }

    void getIRDistances(double dis[5]);
    Position getRobotPosition();

    void updateSettings(SETTINGS settings);

    void init();

    bool mSimulateMode;
    bool mIgnoreObstacle;

  private:
    void check_states();

    double v, w;

    bool unsafe;
    bool danger;

    int m_state;
    double m_right_ticks, m_left_ticks;

  private:
    VelocityController m_Controller;
    //   Robot robot;
    RearDriveRobot robot;

    double d_unsafe;
    Input m_input;
    Output m_output;
};

#endif /* _DRIVE_SUPERVISOR_H_ */
