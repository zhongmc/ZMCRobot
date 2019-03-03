
#ifndef _BALANCE_SUPERVISOR_H_
#define _BALANCE_SUPERVISOR_H_

#include <Arduino.h>

#include "Controller.h"
#include "Robot.h"
#include "BalanceRobot.h"

#include "BalanceController.h"
#include "BalanceSpeedController.h"
#include "VelocityController.h" //w controller

#include <CurieIMU.h>

#include <MadgwickAHRS.h>

#include "kalmanFilter.h"

//#include "MyMPU6050.h"
//#include "MPU6050.h"

#include <Kalman.h>

#define GYRO_RATE 200

class BalanceSupervisor
{
public:
  BalanceSupervisor();
  void execute(long left_ticks, long right_ticks, double dt);
  void reset(Vector goal, double v, double d_fw);
  void reset(long leftTicks, long rightTicks);
  void resetRobot();

  void resetKalman();

  void setGoal(double v, double w);
  void stopDrive();

  void init();
  void updateSettings(SETTINGS settings);
  SETTINGS getSettings(byte settingsType);

  void setBalanceCtrlParam(double val, int idx)
  {
    m_BalanceController.setCtrlParam(val, idx);
  }

  void setSpeedCtrlParam(double val, int idx)
  {
    // m_SpeedController.setCtrlParam(val, idx);
  }

  void getRobotInfo()
  {
    Serial.println("Balance robot info, current state:");
    Serial.print("exec time:");
    Serial.print(execTime);
    Serial.print(", max_pwm:");
    Serial.println(max_pwm);

    // Serial.print(", pwm_diff:");
    // Serial.print(pwm_diff);
    // Serial.print(", pwm_zero:");
    // Serial.println(pwm_zero);

    Serial.print("pwmInfo(b,s,wl，wr):");
    Serial.print(mBalancePWM);
    Serial.print(",");
    // Serial.print(mSpeedPWM);
    // Serial.print(",");
    Serial.print(mwPWM_L);
    Serial.print(",");
    Serial.println(mwPWM_R);

    Serial.print("Input(v, theta):");
    Serial.print(m_input.v);
    Serial.print(", ");
    Serial.println(m_input.theta);

    robot.getRobotInfo();
    Serial.println("balance ");
    m_BalanceController.PrintInfo();
    // Serial.print("velocity ");
    // m_SpeedController.PrintInfo();
    Serial.println("Diff:");
    m_thetaController.PrintInfo();
  }

  //        void setGoal(double x, double y, int theta);
  //the target to go!
  Vector m_Goal;

  Position getRobotPosition();
  //will return the pitch/angle of the robot
  void getIRDistances(double dis[5]);

  //filter paramaters
  double KG_ANG;

  PWM_OUT pwm;
  double mBalancePWM, mSpeedPWM, mwPWM_L, mwPWM_R;
  Vel mVel;

  int pwm_diff, pwm_zero, max_pwm;
  double wheelSyncKP;

  int angleType; //the angle used to control balance 0 sensor 1 kalman 2 es
  void setAngleType(int val)
  {
    angleType = val;
    Serial.print("Change angle type to:");
    Serial.println(val);
  }

  bool mSimulateMode;
  bool mIgnoreObstacle;

private:
  void check_states();

  int m_state;

  //传感器角度（atan（ax/ay）, kalman, madgwick filter, Kalman1
  double m_sensor_angle, m_kalman_angle, m_km_angle; //m_madgwick_angle
  double m_gyro, m_kalman_gyro;

  double KG, m_x_angle; // m_x_angle 通过融合滤波得到，用于判断小车被提起
  //     MPU6050 accelgyro;
  //        MyMPU6050 mpu6050;

  void readIMU(double dt);

  double convertRawAcceleration(int aRaw);
  double convertRawGyro(int gRaw);

  //一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
  // angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
  double estima_cal(double angle, double g_angle, double gyro, double dt, double KG);

  bool layingDown, hangUp;

  Madgwick filter; //, filter2;
  Kalman kalman;
  KalmanFilter km;

  bool progress_made;
  bool at_goal;
  bool at_obstacle;
  bool unsafe;

  //used to count the robot velocity
  double m_per_tick;
  long prev_left_ticks, prev_right_ticks;

  double m_right_ticks, m_left_ticks;

  int speedCounter;

private:
  // Robot(double R, double L, double ticksr, double maxRpm double minRpm)
  // Robot robot;

  BalanceRobot robot;

  double normalize(double in, double limit);

  void sendIMUInfo();

  long execTime;

  BalanceController m_BalanceController;
  BalanceSpeedController m_SpeedController;
  VelocityController m_thetaController;

  double d_fw; //distance to follow wall
  double d_stop;
  double d_at_obs;
  double d_unsafe;
  double d_prog;

  Input m_input;
  Output m_output;
};

#endif /* _BALANCE_SUPERVISOR_H_ */
