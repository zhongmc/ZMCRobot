#ifndef _ZMC_ROBOT_H_
#define _ZMC_ROBOT_H_

#define BALANCE_CAR  1
#define DRIVE_CAR 2

#define CAR_TYPE  BALANCE_CAR  //DRIVE_CAR BALANCE_CAR

#define MAX_ULTRASONIC_DIS 1
#define MAX_IRSENSOR_DIS 0.3
#define SPEED_DOWN_DIS 0.2     //distance to goal to speed down 
#define DIS_SPEED_DOWN_SCALE 10
#define W_SPEED_DOWN_SCALE 1

long readLeftEncoder();
long readRightEncoder();

void MoveLeftMotor( int PWM );
void MoveRightMotor( int PWM );
void StopMotor();

void setGoal(double x, double y, int theta);
void startGoToGoal();
void stopRobot();
void ResetRobot();
void SetSimulateMode(bool sm );
void setDriveGoal(double v, double w);
void startDrive();
void stopAndReset();
void startBalance();
  
#endif






