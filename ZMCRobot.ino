#include "ZMCRobot.h"
#include <CurieBLE.h>
#include <Arduino.h>

#include "CurieTimerOne.h"
#include "IRSensor.h"
#include "Robot.h"

#if CAR_TYPE == DRIVE_CAR
#include "Supervisor.h"
#include "DriveSupervisor.h"
#else
#include "MyMPU6050.h"
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include "BalanceSupervisor.h"
//#include "Kalman.h"
#endif

#include "BlinkLed.h"

//#include "MyKey.h"
//#include "MyMenu.h"

#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 11
#define ULTRASONIC_TRIG 10

//states
#define STATE_IDLE 0
#define STATE_MENU 1
#define STATE_DRIVE 2
#define STATE_GOTOGOAL 3
#define STATE_BALANCE 4
#define STATE_CALIBRATE 5
#define STATE_STEP_RESP 6

#define START_KEY 0
#define LEFT_KEY 1
#define RIGHT_KEY 2
#define RET_KEY 3

#define START_KEY_PIN 11
#define LEFT_KEY_PIN 10
#define RIGHT_KEY_PIN 9
#define RET_KEY_PIN 8

byte currentState = STATE_IDLE;

//MyKey myKey;

#if CAR_TYPE == DRIVE_CAR
Supervisor supervisor;
DriveSupervisor driveSupervisor;
#else
BalanceSupervisor balanceSupervisor;
#endif

Position pos;
BlinkLed blinkLed;

long trigTime, echoTime;
double ultrasonicDistance;

bool doCheckBattleVoltage = true;
bool openDebug = false;
byte settingsReqQueue[8];
short queueLen = 0;

short testState = 0; //1 turnAround, 2, step Resopnse
short testPWM = 90;

//char *titles[] = {"Self balance", "Cruise", "Speed ", "Start", "Remote by BLE", "To Target", "Target X:", "Target Y:", "Start",
//                         "PID of Balance", "KP: ", "KI: ", "KD: ", "Config", "Calibrate Motor", "balance angle"
//                        };

//menu_item menuItems[17];
//MyMenu menu(&menuItems[0]);

unsigned long millisPrevKey, millisPrev;

//bool backLightOn = false;
// LiquidCrystal_I2C lcd(0x27, 16, 2);

//  GP2Y0A41 = 0,     //4-30cm
//  GP2Y0A21 = 1     //10-80cm

IRSensor irSensor(GP2Y0A41);

// SETTINGS mSettings;
// Madgwick filter;
// unsigned long microsPerReading, microsPrevious;
// double accelScale, gyroScale;

static double batteryVoltage;  // Measured battery level
static uint8_t batteryCounter; // Counter used to check if it should check the battery level

void setup()
{

#if CAR_TYPE == BALANCE_CAR

#endif

  Serial.begin(115200);
  delay(100);
  //lcd.init(); // initialize the lcd
  //lcd.backlight(); //Open the backlight
  //  lcd.noBacklight();
  //  lcd.print(" "); // Print a message to the LCD.
  // showMainTips();

  /*
  Serial.println("Initialze key...");
  myKey.addKey(RET_KEY, RET_KEY_PIN);
  myKey.addKey(RIGHT_KEY, RIGHT_KEY_PIN);
  myKey.addKey(LEFT_KEY, LEFT_KEY_PIN);
  myKey.addKey(START_KEY, START_KEY_PIN);

  myKey.initKey();
*/
  // // start the IMU and filter
  // CurieIMU.begin();
  // CurieIMU.setGyroRate(GYRO_RATE);
  // CurieIMU.setAccelerometerRate(GYRO_RATE);
  // // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);

  // filter.begin(GYRO_RATE);

  initMotor();

  // initialize variables to pace updates to correct rate
  //  microsPerReading = 1000000 / GYRO_RATE;  //25
  //  microsPrevious = micros();

  initBluetooth();
  //  Serial.println(F("Initialze MENU..."));
  //
  //void MyMenu::setKeyId(byte stKey, byte rtKey, byte lKey, byte rKey)
  //  menu.setKeyId(START_KEY, RET_KEY,  LEFT_KEY, RIGHT_KEY);
  //  initMenu();

  //  const int oneSecInUsec = 1000000;   // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  //  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &timedBlinkIsr);  // set timer and callback

  //  Serial.println(sizeof(long));

  testState = 0;
  testPWM = 90;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(ULTRASONIC_ECHO, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // ultr sound echo intterupt
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO), UltrasonicEcho, CHANGE);
#if CAR_TYPE == DRIVE_CAR
  SETTINGS mSettings;
  mSettings.sType = 0;

  mSettings.sType = 0;
  mSettings.kp = 5;
  mSettings.ki = 0.01;
  mSettings.kd = 0.05;

  mSettings.max_rpm = 200;
  mSettings.min_rpm = 40; //45

  mSettings.atObstacle = 0.20; //0.15
  mSettings.unsafe = 0.08;
  mSettings.dfw = 0.25;     //0.25
  mSettings.velocity = 0.3; //0.3

  // supervisor.updateSettings(mSettings);
  // driveSupervisor.updateSettings(mSettings);

  supervisor.init();
  driveSupervisor.init();

#else
  SETTINGS mSettings;
  mSettings.sType = 0;
  mSettings.kp = 30;
  mSettings.ki = 0.0;
  mSettings.kd = 0.02;

  mSettings.max_rpm = 140;
  mSettings.min_rpm = 0; //45
  mSettings.max_pwm = 150;

  mSettings.pwm_zero = 0;
  mSettings.pwm_diff = 0;
  mSettings.angleOff = 0.8;

  mSettings.atObstacle = 0.25; //0.15
  mSettings.unsafe = 0.8;      //0.8
  mSettings.dfw = 0.30;        //0.25
  mSettings.velocity = 0.4;    //0.3

  mSettings.wheelSyncKp = 0;
  balanceSupervisor.updateSettings(mSettings);

  //
  //  mSettings.sType = 2;
  //  mSettings.kp = 30;
  //  mSettings.ki = 0.0;
  //  mSettings.kd = 0.02;
  //  balanceSupervisor.updateSettings(mSettings);
  //
  mSettings.sType = 3;
  mSettings.kp = 200;
  mSettings.ki = 10;
  mSettings.kd = 0.0;
  balanceSupervisor.updateSettings(mSettings);
#endif

  millisPrevKey = millis();
  millisPrev = millisPrevKey; //millis();
}

void loop()
{
  checkSerialData();
  blinkLed.beSureToBlink();
  //ble cmd process
  processSetingsRequire();

  //ultrasonic process
  processUltrasonic();

  checkTurnAroundState();
  checkStepResponseState();

  unsigned long millisNow = millis();
  if (millisNow - millisPrev >= 40)
  {

    millisPrev = millisNow;
    double irDistance[5];

#if CAR_TYPE == DRIVE_CAR
    if (currentState == STATE_GOTOGOAL)
    {
      //report states
      supervisor.getIRDistances(irDistance);
      pos = supervisor.getRobotPosition();
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
    else if (currentState == STATE_DRIVE)
    {
      driveSupervisor.getIRDistances(irDistance);
      pos = driveSupervisor.getRobotPosition();
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
    else if (currentState == STATE_STEP_RESP)
    {
      driveSupervisor.getRobotVel(irDistance);
      sendRobotStateValue(3, pos, irDistance, batteryVoltage);
    }
#else
    if (currentState == STATE_BALANCE)
    {
      balanceSupervisor.getIRDistances(irDistance);
      pos = balanceSupervisor.getRobotPosition();
      sendBalanceRobotStateValue(pos, irDistance, batteryVoltage);
      /*
              Serial.print(balanceSupervisor.pwm.pwm_l);
              Serial.print(",");
              Serial.print(balanceSupervisor.pwm.pwm_r);
              Serial.print(",");

              Serial.print(balanceSupervisor.mVel.vel_l);
              Serial.print(",");
              Serial.print(balanceSupervisor.mVel.vel_r);
              Serial.print(",");

              Serial.print(balanceSupervisor.mBalancePWM );
              Serial.print(",");

              Serial.print(irDistance[0]);
              Serial.print(",");
              Serial.print(irDistance[1]);
              Serial.print(",");
              Serial.println(irDistance[2]);
      */
    }
#endif

    else
    {
      for (int i = 0; i < 5; i++)
      {
        irDistance[i] = irSensor.readDistance(1 + i);
        //delay(5);
      }
      if (ultrasonicDistance < MAX_ULTRASONIC_DIS) //irDistance[2] )
        irDistance[2] = ultrasonicDistance;

      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }

    /*
    if ( LED_ON )
    {
      LED_ON = false;
      digitalWrite( 13, HIGH);
    }
    else
    {
      LED_ON = true;
      digitalWrite( 13, LOW);
    }
*/

    batteryCounter++;
    if (batteryCounter >= 10)
    { // Measure battery every 1s

      if (currentState != STATE_MENU)
      {
        //          lcd.clear();
        //          lcd.setCursor(0, 0);
        //          lcd.print(PIDVAL);
        //
        //          showMainTips();
        //
        //          lcd.setCursor(0, 1);
        //          lcd.print( roll );
        //          lcd.print(" ");
        //          lcd.print(batteryVoltage);
      }

      batteryCounter = 0;
      batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0352771; /// 65.7424242f;
      // v = D * 3.3*(R1+R2)/(R2*1023);  R1 = 46.5 R2 = 4.68 V = d* 0.0352771;
      // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 56k-15k voltage divider - 1023.0/(3.3/(15.0/(15.0+56.0))) = 63.050847458

      if ((batteryVoltage < 9 && batteryVoltage < 7.2) || (batteryVoltage > 9 && batteryVoltage < 11.5)) // && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
      {
        if (doCheckBattleVoltage)
        {
          stopAndReset();
          currentState = STATE_IDLE;
          stopRobot();
          blinkLed.slowBlink();
        }
      }
      else
        blinkLed.normalBlink();
    }
  }
}

#if CAR_TYPE == DRIVE_CAR

void setGoal(double x, double y, int theta)
{
  supervisor.setGoal(x, y, theta);
}

void startGoToGoal()
{
  if (currentState >= 2)
    return;

  // supervisor.updateSettings(mSettings);
  Serial.print("Start Go to Goal:");
  Serial.print(supervisor.m_Goal.x);
  Serial.print(",");
  Serial.println(supervisor.m_Goal.y);

  // to test set goal y to 0
  supervisor.reset(readLeftEncoder(), readRightEncoder());
  currentState = STATE_GOTOGOAL;

  const int oneSecInUsec = 1000000;                     // A second in mirco second unit.
  CurieTimerOne.start(oneSecInUsec / 50, &goToGoalIsr); // set timer and callback //the controller loop need 30ms to exec
}

void goToGoalIsr()
{
  supervisor.execute(readLeftEncoder(), readRightEncoder(), 0.02);
}

void ResetRobot()
{
  supervisor.resetRobot();
  driveSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

void startDrive()
{
  // if ( currentState >= 2 )
  //   return;

  Serial.println("Start drive!");
  currentState = STATE_DRIVE;

  // to test set goal y to 0
  driveSupervisor.reset(readLeftEncoder(), readRightEncoder());
  //   currentState = STATE_DRIVE;

  const int oneSecInUsec = 1000000; // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  CurieTimerOne.start(oneSecInUsec / 50, &driveIsr); // set timer and callback
}

void driveIsr()
{
  driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.02);
}

void SetSimulateMode(bool val)
{
  supervisor.mSimulateMode = val;
  driveSupervisor.mSimulateMode = val;
  if (val)
  {
    doCheckBattleVoltage = false;
    Serial.println("Set to simulate mode!");
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate mode!");
  }
}

void SetIgnoreObstacle(bool igm)
{
  Serial.println("set ignore obstacle mode: " + igm);
  supervisor.mIgnoreObstacle = igm;
  driveSupervisor.mIgnoreObstacle = igm;
}

#else

void startBalance()
{
  if (currentState >= 2)
    return;

  blinkLed.fastBlink();

  currentState = STATE_BALANCE;

  Serial.print("Start balance!");
  balanceSupervisor.reset(readLeftEncoder(), readRightEncoder());
  const int oneSecInUsec = 1000000;                           // A second in mirco second unit.
  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &balanceIsr); // set timer and callback
}

void balanceIsr()
{
  balanceSupervisor.execute(readLeftEncoder(), readRightEncoder(), 1 / (double)GYRO_RATE);
}

void SetIgnoreObstacle(bool igm)
{
  Serial.println("set ignore obstacle mode: " + igm);

  balanceSupervisor.mIgnoreObstacle = igm;
}

void SetSimulateMode(bool val)
{
  balanceSupervisor.mSimulateMode = val;
  if (val)
  {
    doCheckBattleVoltage = false;
    Serial.println("Set to simulate mode!");
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate mode!");
  }
}

void ResetRobot()
{
  balanceSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

#endif

void stopRobot()
{
  blinkLed.normalBlink();
  currentState = STATE_IDLE;
  CurieTimerOne.kill();
  stopAndReset();
}

void setDriveGoal(double v, double w)
{
#if CAR_TYPE == DRIVE_CAR
  if (currentState == STATE_DRIVE)
  {
    if (abs(v) < 0.001 && abs(w) < 0.01) //stop
      stopRobot();
    else
      driveSupervisor.setGoal(v, w);
  }
  else
  {
    startDrive();
    driveSupervisor.setGoal(v, w);
  }

#else
  if (currentState == STATE_BALANCE)
  {
    balanceSupervisor.setGoal(v, w);
  }
#endif
}

bool waitForEcho = false;
long lastTrigTimer = 0;

void processUltrasonic()
{
  if (waitForEcho)
  {
    if (echoTime > 0)
    {
      ultrasonicDistance = 0.00017 * echoTime;
      if (ultrasonicDistance > MAX_ULTRASONIC_DIS)
        ultrasonicDistance = MAX_ULTRASONIC_DIS - 0.01;
      waitForEcho = false;
    }
    else if (millis() - lastTrigTimer > 50)
    {
      waitForEcho = false;
      //      Serial.println("ultrasonic No echo ...");
      ultrasonicDistance = MAX_ULTRASONIC_DIS;
    }
  }
  else
  {
    long curTime = millis();
    if (curTime - lastTrigTimer < 20)
      return;
    lastTrigTimer = curTime;
    waitForEcho = true;
    digitalWrite(ULTRASONIC_TRIG, HIGH); //trig the ultrosonic
    //delayMicroseconds(2); //2us
    for (int i = 0; i < 100; i++)
    {
      int k;
      k = i;
    }
    digitalWrite(ULTRASONIC_TRIG, LOW); //trig the ultrosonic
  }
}

//the ultrasonic isr service
void UltrasonicEcho()
{

  int echoSig = digitalRead(ULTRASONIC_ECHO);
  if (echoSig == HIGH)
  {
    trigTime = micros();
    echoTime = 0;
  }
  else
    echoTime = micros() - trigTime;
}

extern long count1, count2;

unsigned long testMillisPrev;
//启动转圈测试，以测定轮距
void startTurnAround(int pwm)
{
  if (testState != 0)
    return;
  testState = 1;
  testPWM = pwm;

  count1 = 0;
  count2 = 0;
  Serial.print("Start turn around test: ");
  Serial.println(pwm);
  testMillisPrev = millis();

  MoveLeftMotor(pwm);
}

//启动阶跃响应测试
void startStepResponse(int pwm)
{

  if (currentState == STATE_STEP_RESP)
    return;

  if (currentState != STATE_IDLE)
  {
    stopRobot();
  }

  currentState = STATE_STEP_RESP;
  testPWM = pwm;
  count1 = 0;
  count2 = 0;
  Serial.print("Start Step response test: ");
  Serial.println(pwm);
  testMillisPrev = millis();
  MoveMotor(100); //pwm
}

void checkTurnAroundState()
{
  if (testState != 1)
    return;

  unsigned long curMillis = millis();
  if (curMillis - testMillisPrev >= 20)
  {
    long c1, c2;
    c1 = count1;
    c2 = count2;

    Serial.print(c1);
    Serial.print(",");
    Serial.println(c2);

    if (c1 > 1676 || c2 > 1828)
    {
      stopRobot();
      testState = 0; //over
    }

    testMillisPrev = curMillis;
  }
}

void checkStepResponseState()
{
  if (currentState != STATE_STEP_RESP)
    return;

  unsigned long curMillis = millis();
  if (curMillis - testMillisPrev >= 20)
  {
    long c1, c2;
    c1 = count1;
    c2 = count2;
    driveSupervisor.updateRobot(c1, c2, 90, 0.02);
    double vels[5];
    driveSupervisor.getRobotVel(vels);
    c1 = (int)(vels[0] * 10000.0);
    c2 = (int)(vels[1] * 10000.0);
    Serial.print(c1);
    Serial.print(",");
    Serial.println(c2);
    testMillisPrev = curMillis;
  }
}
