
#ifndef _BALANCE_SUPERVISOR_H_
#define _BALANCE_SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "BalanceRobot.h"

#include "BalanceController.h"
#include "BalanceSpeedController.h"
#include "VelocityController.h"  //w controller

#include <CurieIMU.h>
#include <MadgwickAHRS.h>

//#include "MyMPU6050.h"
//#include "MPU6050.h"

#include <Kalman.h>

#define GYRO_RATE   100

class BalanceSupervisor {
  public:
    BalanceSupervisor();
    void execute(long left_ticks, long right_ticks, double dt);
    void reset(Vector goal, double v, double d_fw);
    void reset(long leftTicks, long rightTicks);
    void resetRobot();

    void resetKalman();

    void setGoal(double v, double w);
    void stopDrive();

    void updateSettings(SETTINGS settings);
    SETTINGS getSettings( byte settingsType );

    void setBalanceCtrlParam(double val, int idx)
    {
      m_BalanceController.setCtrlParam( val, idx);
    }

    void setSpeedCtrlParam(double val, int idx)
    {
      m_SpeedController.setCtrlParam(val, idx);
    }


    void getRobotInfo()
    {
      Serial.println("Balance robot info, current state:");
      Serial.print("exec time:");
      Serial.print(execTime);
      Serial.print(", max_pwm:");
      Serial.print(max_pwm);

      Serial.print(", pwm_diff:");
      Serial.print(pwm_diff);
      Serial.print(", pwm_zero:");
      Serial.println( pwm_zero);

      Serial.print("pwmInfo(b,s,wl，wr):");
      Serial.print(mBalancePWM);
      Serial.print(",");

      Serial.print(mSpeedPWM);
      Serial.print(",");
      Serial.print(mwPWM_L);
      Serial.print(",");
      Serial.println(mwPWM_R);

      Serial.print("Input(v, theta):");
      Serial.print(m_input.v);
      Serial.print(", ");
      Serial.println(m_input.theta);


      robot.getRobotInfo();
      Serial.print("balance ");
      m_BalanceController.PrintInfo();
      Serial.print("velocity ");
      m_SpeedController.PrintInfo();
      Serial.print("Diff:");
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

    int angleType;  //the angle used to control balance 0 sensor 1 kalman 2 es
    void setAngleType(int val)
    {
      angleType = val;
      Serial.print("Change angle type to:");
      Serial.println(val);
    }

    bool mSimulateMode;

  private:
    void check_states();

    int m_state;
    double  m_sensor_angle, m_estima_angle, g_fGravityAngle;
    double m_gyro;

    //     MPU6050 accelgyro;

    //        MyMPU6050 mpu6050;

    double readIMU();
    double convertRawAcceleration(int aRaw);
    double convertRawGyro(int gRaw);

    double estima_cal (double estima, double metron, double KG);

    bool layingDown;

    Madgwick filter; //, filter2;

    Kalman kalman;



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

    long execTime;

    BalanceController m_BalanceController;
    BalanceSpeedController m_SpeedController;
    VelocityController m_thetaController;

    double d_fw;  //distance to follow wall
    double d_stop;
    double d_at_obs;
    double d_unsafe;
    double d_prog;

    Input m_input;
    Output m_output;
};


#endif /* _BALANCE_SUPERVISOR_H_ */





