#include "ZMCRobot.h"
#include <CurieBle.h>

#if CAR_TYPE == DRIVE_CAR
#include "Supervisor.h"
#include "DriveSupervisor.h"
#else
#include "BalanceSupervisor.h"
//#include "Kalman.h"
#endif

#if CAR_TYPE == DRIVE_CAR
extern Supervisor supervisor;
extern DriveSupervisor driveSupervisor;
#else
extern BalanceSupervisor balanceSupervisor;
#endif

extern long trigTime, echoTime;
extern double ultrasonicDistance;

static char comData[30];
int comDataCount;

//extern Supervisor supervisor;
extern double gp2y0a41[3][4];
extern long count1, count2;

extern bool openDebug;

extern double batteryVoltage;

void checkSerialData()
{

  //read speed setting from serial
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      char ch = Serial.read();
      comData[comDataCount++] = ch;
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        processCommand(comData, comDataCount);
        comDataCount = 0;
      }
      if (comDataCount > 20) //some error
      {
        Serial.print("UnKnow Command:");
        comData[comDataCount] = 0;
        Serial.println(comData);
        comDataCount = 0;
      }
    }
  }
}

//  Serial.println("\r\n\r\n***** BLE Commands *******************************");
//  Serial.println(" ST stop!");
//  Serial.println(" RS reset robot");
//  Serial.println(" RP[type] require for settings, 1,2,3,4,5");
//  Serial.println(" RT[type] Set robot type 0 normal 1 balance");
//  Serial.println(" SM[val] Set simulate mode 1 true");
//  Serial.println(" GO Start goto goal");
//  Serial.println(" GG[][][] Set goto goal goan x,y,Q");
//  Serial.println(" GD Start drive mode");
//  Serial.println(" GB Start balance mode");
//  Serial.println(" SD[v][w] set drive goal");
//  Serial.println(" BS stop balance drive or goto goal\r\n\r\n");
//
// Serial.println(" ?; print this info");
// Serial.println(" gr[]; controller infos");
// Serial.println(" sb; start balance");
// Serial.println(" st; stop robot");
// Serial.println(" gd; distance of ultraSonic");
// Serial.println(" od; open debug");
// Serial.println(" cd; close debug");
// Serial.println(" ci; read count");

// Serial.println(" mm[pwm]; move motor with pwm");
// Serial.println(" ml[pwm]; move left motor with pwm");
// Serial.println(" mr[pwm]; move right motor with pwm");
// Serial.println(" gg[xd]; start go to goal");

// Serial.println(" sm[0/1]; simulate mode");
// Serial.println(" io[0/1]; ignore obstacle");
// Serial.println(" rs; reset");
// Serial.println(" tl; turn around test left");
// Serial.println(" tr; turn around test right");

//  Serial.println(" bp[]; balance kp");
//  Serial.println(" bi[]; balance ki");
//  Serial.println(" bd[]; balance kd");
//
//  Serial.println(" vp[]; velocity kp");
//  Serial.println(" vi[]; velocity ki");
//  Serial.println(" vd[]; velocity kd");

void processCommand(char *buffer, int bufferLen)
{
  *(buffer + bufferLen) = 0;
  Serial.println(buffer);

  if (bufferLen <= 2)
    return;
  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  if (ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n\r\n====");
    Serial.print("Voltage:");
    Serial.print(batteryVoltage);
    Serial.print(", ultrasonic dis:");
    Serial.println(ultrasonicDistance);
#if CAR_TYPE == DRIVE_CAR
    Serial.println("\r\n====");
    supervisor.getRobotInfo();
    Serial.println("\r\n====");
    driveSupervisor.getRobotInfo();
#else
    Serial.println("\r\n====");
    balanceSupervisor.getRobotInfo();
#endif
  }

  // else if (ch0 == 'o' && ch1 == 'd') //open debug
  // {
  //   openDebug = true;
  // }
  // else if (ch0 == 'c' && ch1 == 'd') //close debug
  // {
  //   openDebug = false;
  // }

  // else if (ch0 == 'g' && ch1 == 'd')
  // {
  //   Serial.print("Distance of ultraSonic:");
  //   if (echoTime > 0)
  //   {
  //     float dist = 0.017 * echoTime;
  //     //     float dist = 17000/echoTime;
  //     Serial.println(dist);
  //   }
  //   else
  //   {
  //     Serial.println("NA");
  //   }
  // }

  else if (ch0 == 's' && ch1 == 't') //stop
  {
    Serial.println("Stop!");
    stopRobot();
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
    printCountInfo();
  }
  else if (ch0 == 'm' && ch1 == 'm') // move motor
  {
    int pwm = atoi(buffer + 2);
    motorSpeed(pwm);
    MoveMotor(0);
  }

  else if (ch0 == 'm' && ch1 == 'l') // move left motor
  {
    int pwm = atoi(buffer + 2);
    // Serial.print("Move left motor: ");
    // Serial.println(pwm);

    // Serial.print("C1=");
    // Serial.print(count1);
    // Serial.print(", C2=");
    // Serial.println(count2);
    // Serial.print("time:");
    // Serial.println(millis());

    printCountInfo();
    MoveLeftMotor(pwm);
  }
  else if (ch0 == 's' && ch1 == 'p') //speed test
  {
    int pwm0, pwm1, step = 0;
    pwm0 = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');
    pwm1 = atoi(buf + 1);
    buf = strchr((buf + 1), ',');
    step = atoi(buf);

    // sscanf((buffer + 2), "%d,%d,%d", &pwm0, &pwm1, &step);
    Serial.print("SP:");
    Serial.print(pwm0);
    Serial.print(",");
    Serial.print(pwm1);
    Serial.print(",");
    Serial.println(step);
    if (step == 0)
      step = 10;

    speedTest(pwm0, pwm1, step);
  }

  else if (ch0 == 'm' && ch1 == 'r') // move right motor
  {
    int pwm = atoi(buffer + 2);
    // Serial.print("Move right motor: ");
    // Serial.println(pwm);

    // Serial.print("C1=");
    // Serial.print(count1);
    // Serial.print(", C2=");
    // Serial.println(count2);
    // Serial.print("current time:");
    // Serial.println(millis());
    printCountInfo();
    MoveRightMotor(pwm);
  }
  else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test
  {
    int pwm = atoi(buffer + 2);
    startTurnAround(pwm);
  }
  else if (ch0 == 'g' && ch1 == 'g') //go to goal
  {

    count1 = 0;
    count2 = 0;

    supervisor.reset(0, 0);

    float d = atof(buffer + 2);
    setGoal(d, 0, 0);
    startGoToGoal();
  }

  else if (ch0 == 's' && ch1 == 'm') //simulate mode
  {
    int val = atoi(buffer + 2);
    SetSimulateMode(val);

    //     if (val == 1)
    //         SetSimulateMode(true);
    // else
    //   SetSimulateMode(false);
  }
  else if (ch0 == 'i' && ch1 == 'o') //ignore atObstacle
  {
    int val = atoi(buffer + 2);
    SetIgnoreObstacle(val);

    // if (val == 1)
    //   SetIgnoreObstacle(true);
    // else
    //   SetIgnoreObstacle(false);
  }

  else if (ch0 == 'r' && ch0 == 's') //RESET
  {
    ResetRobot();
  }

#if CAR_TYPE == BALANCE_CAR

  else if (ch0 == 's' && ch1 == 'b') //start balance
  {
    Serial.println("Start balance!");
    startBalance();
  }

  else if (ch0 == 'b') //setting  balance values
  {
    double val;
    if (ch1 == 'p')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KP to:");
      Serial.println(val);

      balanceSupervisor.setBalanceCtrlParam(val, 0);
    }
    else if (ch1 == 'i')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KI to:");
      Serial.println(val);
      balanceSupervisor.setBalanceCtrlParam(val, 1);
    }
    else if (ch1 == 'd')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KD to:");
      Serial.println(val);
      balanceSupervisor.setBalanceCtrlParam(val, 2);
    }
  }

  else if (ch0 == 'c' && ch1 == 'i')
  {
    //  CalibrateIMU();
  }

  else if (ch0 == 'v') //setting  speed values
  {
    double val;
    if (ch1 == 'p')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set speed KP to:");
      Serial.println(val);

      balanceSupervisor.setSpeedCtrlParam(val, 0);
    }
    else if (ch1 == 'i')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set Speed KI to:");
      Serial.println(val);
      balanceSupervisor.setSpeedCtrlParam(val, 1);
    }
    else if (ch1 == 'd')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set Speed KD to:");
      Serial.println(val);
      balanceSupervisor.setSpeedCtrlParam(val, 2);
    }
  }
#endif
}

/*
#if CAR_TYPE == BALANCE_CAR
void CalibrateIMU()
{
    Serial.println("Internal sensor offsets BEFORE calibration( xAcc,yAcc,zAcc, xGyro, yGyro, zGyro ...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

  
}
#endif
*/

void printCountInfo()
{
  Serial.print(millis());
  Serial.print(',');
  Serial.print(count1);
  Serial.print(',');
  Serial.println(count2);

  // Serial.print("C1=");
  // Serial.print(count1);
  // Serial.print(", C2=");
  // Serial.println(count2);
  // Serial.print("time:");
  // Serial.println(millis());
}

void speedTest(int pwm0, int pwm1, int step)
{
  // long c1, c2, lt;
  for (int i = pwm0; i < pwm1; i += step)
  {
    motorSpeed(i);
  }
  MoveMotor(0);
}

void motorSpeed(int pwm)
{
  long c1, c2, lt;
  MoveMotor(pwm);
  Serial.print(pwm);
  Serial.print(',');
  delay(300);
  c1 = count1;
  c2 = count2;
  lt = millis();
  delay(1000);

  c1 = count1 - c1;
  c2 = count2 - c2;
  lt = millis() - lt;
  Serial.print(lt);
  Serial.print(',');
  Serial.print(c1);
  Serial.print(',');
  Serial.println(c2);
}