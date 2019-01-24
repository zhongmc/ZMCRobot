#include "ZMCRobot.h"
#include <CurieBle.h>

#include "robot.h"

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

extern bool doCheckBattleVoltage; // = true;
extern bool openDebug;            // = false;
extern byte settingsReqQueue[8];
extern short queueLen; // = 0;

BLEPeripheral blePeripheral;
// BLE Peripheral Device (the board you're programming)
BLEService zmcRobotService("3A37"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"

BLECharacteristic zmcRobotSettingsChar("3A38",
                                       // standard 16-bit characteristic UUID
                                       BLERead | BLEWrite | BLENotify, 18); //KP KI KD atObstacle unsafe velocity

BLECharacteristic zmcRobotDriveChar("3A39",
                                    // standard 16-bit characteristic UUID
                                    BLERead | BLEWrite, 16); //CMD:2, datas;    speed tl tr | BLENotify

BLECharacteristic zmcRobotStateChar("3A3A",
                                    // standard 16-bit characteristic UUID
                                    BLERead | BLENotify, 19); // x,y,theta,irdis 0-4, volt of bat | BLENotify

bool bleConnected = false;

// remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications

void initBluetooth()
{

  Serial.println("init Bluetooth device ...");

  bleConnected = false;
  /*The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("ZMC Robot");
  blePeripheral.setAdvertisedServiceUuid(zmcRobotService.uuid());
  // add the service UUID
  blePeripheral.addAttribute(zmcRobotService);

  // add the Heart Rate Measurement characteristic
  blePeripheral.addAttribute(zmcRobotSettingsChar);
  blePeripheral.addAttribute(zmcRobotDriveChar);
  blePeripheral.addAttribute(zmcRobotStateChar);

  /* Now activate the BLE device. It will start continuously transmitting BLE
    advertising packets and will be visible to remote BLE central devices
    until it receives a new connection */

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  zmcRobotSettingsChar.setEventHandler(BLEWritten, configCharacteristicWritten);
  zmcRobotDriveChar.setEventHandler(BLEWritten, driveCharacteristicWritten);

  blePeripheral.begin();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

void blePeripheralConnectHandler(BLECentral &central)
{

  bleConnected = true;
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());

  Serial.print("UUID:");
  Serial.print(zmcRobotService.uuid());

  //  sendRobotConfigValue();
}

void blePeripheralDisconnectHandler(BLECentral &central)
{

  bleConnected = false;
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void configCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // central wrote new value to characteristic, update LED
  Serial.println("config Characteristic event, written: ");
  setConfigValue(characteristic.value());
}

void driveCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // central wrote new value to characteristic, update LED
  Serial.print("drive Characteristic event,cmd:");
  //the first two chas as CMD

  unsigned char *data = (unsigned char *)characteristic.value();

  char cmd[3];
  cmd[2] = '\0';
  cmd[0] = data[0];
  cmd[1] = data[1];
  Serial.println(cmd);

  if (cmd[0] == 'S' && cmd[1] == 'T') //stop
  {
    stopRobot();
  }
  else if (cmd[0] == 'R' && cmd[1] == 'P') //Required for settings
  {
    Serial.println("Required for settings Of:");
    Serial.println(data[2]);

    if (queueLen < 7)
      settingsReqQueue[queueLen++] = data[2];
    else
      Serial.println("Queue out of bound!");

    //    requireForSettings = true;
    //    requiredSettingsType = data[2];
  }
  else if (cmd[0] == 'R' && cmd[1] == 'T') //robot type 0 3wheel car ; 1 balance robot
  {
    Serial.print("Set robot type;");
    Serial.println(data[2]);
  }
  else if (cmd[0] == 'C' && cmd[1] == 'B') // check battle voltage
  {
    if (data[2] == 1)
      doCheckBattleVoltage = true;
    else
      doCheckBattleVoltage = false;
  }

  else if (cmd[0] == 'S' && cmd[1] == 'M') //simulate mode
  {
    if (data[2] == 1)
      SetSimulateMode(true);
    else
      SetSimulateMode(false);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'O') //ignore atObstacle
  {
    if (data[2] == 1)
      SetIgnoreObstacle(true);
    else
      SetIgnoreObstacle(false);
  }

  else if (cmd[0] == 'R' && cmd[1] == 'S') //RESET
  {
    ResetRobot();
  }

#if CAR_TYPE == DRIVE_CAR

  else if (cmd[0] == 'G' && cmd[1] == 'O') // action...
  {
    startGoToGoal();
  }

  else if (cmd[0] == 'G' && cmd[1] == 'D') //start drive mode
  {
    startDrive();
  }

  else if (cmd[0] == 'G' && cmd[1] == 'G') // Go To Goal: x, y, theta
  {
    double x, y;
    int theta;
    x = byteToFloat((byte *)(data + 2), 100);
    y = byteToFloat((byte *)(data + 4), 100);
    theta = byteToInt((byte *)(data + 6));

    setGoal(x, y, theta);

    Serial.print("Go to Goal:");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(theta);
  }

  else if (cmd[0] == 'M' && cmd[1] == 'G') //go to goal
  {
    setGoal(1, 0, 0);
  }

  else if (cmd[0] == 'S' && cmd[1] == 'R') //step response
  {
    startStepResponse(90);
  }
  else if (cmd[0] == 'T' && cmd[1] == 'L') //turn around
  {
    startTurnAround(100);
  }
  else if (cmd[0] == 'T' && cmd[1] == 'R') //turn around
  {
    startTurnAround(-100);
  }
#else
  else if (cmd[0] == 'G' && cmd[1] == 'B') //start Balance mode
  {
    startBalance();
  }
  else if (cmd[0] == 'B' && cmd[1] == 'S') //stop balance drive or goto goal
  {
    balanceSupervisor.stopDrive();
  }
  else if (cmd[0] == 'A' && cmd[1] == 'S') //angle select 0 sensor 1 kalman 2 es
  {
    balanceSupervisor.setAngleType(data[2]);
  }
#endif

  else if (cmd[0] == 'S' && cmd[1] == 'D') //set drive Goal
  {
    double v, w;
    v = byteToFloat((byte *)(data + 2), 100);
    w = byteToFloat((byte *)(data + 4), 100);
    setDriveGoal(v, w);
  }
}

void setConfigValue(const unsigned char *cfgArray)
{
  int settingsType = (int)cfgArray[0];
  Serial.print("config the robot via ble:");
  Serial.println(settingsType);

  SETTINGS settings;
  settings.sType = settingsType;

  if (settingsType == 1 || settingsType == 2 || settingsType == 3)
  {
    settings.kp = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.ki = byteToFloat((byte *)(cfgArray + 3), 1000);
    settings.kd = byteToFloat((byte *)(cfgArray + 5), 1000);
    Serial.print("KP:");
    Serial.print(settings.kp);
    Serial.print(" KI:");
    Serial.print(settings.ki);
    Serial.print(" KD:");
    Serial.println(settings.kd);
  }
  else if (settingsType == 4)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.dfw = byteToFloat((byte *)(cfgArray + 5), 100);
    settings.velocity = byteToFloat((byte *)(cfgArray + 7), 100);

    settings.max_rpm = byteToInt((byte *)(cfgArray + 9));
    settings.min_rpm = byteToInt((byte *)(cfgArray + 11));

    settings.radius = byteToFloat((byte *)(cfgArray + 13), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 15), 1000);

    // settings.pwm_diff = (int)cfgArray[13]; //byteToInt((byte *)(cfgArray + 13) );
    // settings.pwm_zero = (int)cfgArray[14];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 15), 100);

    Serial.print(" atObstacle:");
    Serial.print(settings.atObstacle);

    Serial.print(" unsafe:");
    Serial.print(settings.unsafe);
    Serial.print(" dfw:");
    Serial.print(settings.dfw);

    Serial.print(" v:");
    Serial.print(settings.velocity);
    Serial.print(" max_rpm:");
    Serial.print(settings.max_rpm);
    Serial.print(" min_rpm:");
    Serial.print(settings.min_rpm);
    Serial.print(" radius:");
    Serial.print(settings.radius);
    Serial.print(" length:");
    Serial.println(settings.length);

    // Serial.print(" pwm_diff:");
    // Serial.print(settings.pwm_diff);
    // Serial.print(" pwm_zero:");
    // Serial.print(settings.pwm_zero);
    // Serial.print(" angle_off:");
    // Serial.println(settings.angleOff);
  }
  else if (settingsType == 5)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.max_pwm = byteToInt((byte *)(cfgArray + 5));
    settings.pwm_zero = (int)cfgArray[7];
    settings.pwm_diff = (int)cfgArray[8];
    settings.angleOff = byteToFloat((byte *)(cfgArray + 9), 100);
    settings.wheelSyncKp = byteToFloat((byte *)(cfgArray + 11), 100);

    Serial.print(" atObstacle:");
    Serial.print(settings.atObstacle);

    Serial.print(" unsafe:");
    Serial.print(settings.unsafe);
    Serial.print(" max_pwm:");
    Serial.print(settings.max_pwm);
    Serial.print(" pwm_zero:");
    Serial.print(settings.pwm_zero);
    Serial.print(" pwm_diff:");
    Serial.print(settings.pwm_diff);
    Serial.print(" angle_off:");
    Serial.print(settings.angleOff);
    Serial.print(" wheelSyncKP:");
    Serial.println(settings.wheelSyncKp);
  }

#if CAR_TYPE == DRIVE_CAR
  if (settingsType == 1 || settingsType == 4)
  {
    supervisor.updateSettings(settings);
    driveSupervisor.updateSettings(settings);
  }
#else
  if (settingsType == 2 || settingsType == 3 || settingsType == 5)
  {
    balanceSupervisor.updateSettings(settings);
  }
#endif

  // setSettings(settings);
  //  updateConfigToMenu();
}

/*
void sendRobotConfigValue()
{

  if ( !bleConnected )
    return;

  Serial.println("Send the robot settings!");
  byte settingsArray[18];

  floatToByte(settingsArray, mSettings.kp, 100 );
  floatToByte(settingsArray+2, mSettings.ki, 1000 );
  floatToByte(settingsArray+4, mSettings.kd, 1000 );
  floatToByte(settingsArray+6, mSettings.atObstacle, 100 );  
  floatToByte(settingsArray+8, mSettings.unsafe, 100 );  
  floatToByte(settingsArray+10, mSettings.dfw, 100 );  
  floatToByte(settingsArray+12, mSettings.velocity, 100 );

  intToByte(settingsArray+14, mSettings.max_rpm );
  intToByte(settingsArray+16, mSettings.min_rpm );
  
  zmcRobotSettingsChar.setValue( settingsArray, 18 );
}
*/

void intToByte(byte *arrayBuf, int val)
{
  if (val > 0)
  {
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val;
  }
  else
  {
    val = -val;
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val | 0x80;
  }
}

void floatToByte(byte *arrayBuf, double val, double scale)
{
  int tmp = (int)(val * scale);
  if (tmp > 0)
  {
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp;
  }
  else
  {
    tmp = -tmp;
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp | 0x80;
  }
}

double byteToFloat(byte *arrayBuf, double scale)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;
  return (double)val / scale;
}

int byteToInt(byte *arrayBuf)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;

  return val;
}

//type angle1,2,3,voltage
void sendBalanceRobotStateValue(Position pos, double irDistance[5], double voltage)
{
  if (!bleConnected)
    return;
  byte buf[19];
  memset(buf, 0, 19);
  buf[0] = 2;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  scale = 100;

  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
  }
  floatToByte(buf + 17, voltage, scale);
  bool ret = zmcRobotStateChar.setValue(buf, 19);
  if (!ret)
  {
    Serial.println("Failed to write balance state character!");
  }
}

//type, x, y, theta, d0,d1,d2,d3,d4,voltage
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage)
{
  if (!bleConnected)
    return;

  byte buf[19];
  memset(buf, 0, 19);

  buf[0] = stateType;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  //    Serial.print( voltage);
  //    Serial.print( ", " );

  scale = 100;
  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
    //  Serial.print( irDistance[i] );
    //    Serial.print( "," );
  }
  //    Serial.println( ";" );

  floatToByte(buf + 17, voltage, scale);

  bool ret = zmcRobotStateChar.setValue(buf, 19);
  if (!ret)
  {
    Serial.println("Failed to write state character!");
  }
}

void processSetingsRequire()
{
  if (queueLen == 0)
    return;
  byte sType = settingsReqQueue[0];
  for (int i = 0; i < queueLen - 1; i++)
    settingsReqQueue[i] = settingsReqQueue[i + 1];
  queueLen--;

  SETTINGS settings;
  settings.sType = sType;
  // // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: settings for robot; 5: settings for balance robot;

#if CAR_TYPE == DRIVE_CAR
  if (sType == 1 || sType == 4)
  {
    settings = supervisor.getSettings(sType);
  }
  else if (sType == 2)
  {
    settings.kp = 18;
    settings.ki = 0.0;
    settings.kd = 2.0;
  }
  else if (sType == 3)
  {
    settings.kp = 6;
    settings.ki = 0.01;
    settings.kd = 0.0;
  }
  else if (sType == 5)
  {
    settings.atObstacle = 0.20; //0.15
    settings.unsafe = 0.08;
    settings.dfw = 0.25;     //0.25
    settings.velocity = 0.5; //0.3
    settings.max_rpm = 140;
    settings.min_rpm = -140; //45

    settings.pwm_diff = 0;
    settings.pwm_zero = 5;
    settings.angleOff = 0;
  }
#else
  if (sType == 2 || sType == 3 || sType == 5)
  {
    settings = balanceSupervisor.getSettings(sType);
  }
  else
  {
    settings.sType = 0;
    settings.kp = 5;
    settings.ki = 0.01;
    settings.kd = 0.05;

    settings.atObstacle = 0.20; //0.15
    settings.unsafe = 0.08;
    settings.dfw = 0.25;     //0.25
    settings.velocity = 0.5; //0.3
    settings.max_rpm = 200;
    settings.min_rpm = 10; //45
  }
#endif
  SendSettings(settings);
}

void SendSettings(SETTINGS settings)
{
  int settingsType = settings.sType;

  Serial.print("Send settings:");
  Serial.println(settingsType);

  byte settingsArray[18];
  settingsArray[0] = (byte)settingsType;
  int len = 7;
  if (settingsType == 1 || settingsType == 2 || settingsType == 3)
  {

    floatToByte(settingsArray + 1, settings.kp, 100);
    floatToByte(settingsArray + 3, settings.ki, 1000);
    floatToByte(settingsArray + 5, settings.kd, 1000);
  }
  else if (settingsType == 4)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    floatToByte(settingsArray + 5, settings.dfw, 100);
    floatToByte(settingsArray + 7, settings.velocity, 100);

    intToByte(settingsArray + 9, settings.max_rpm);
    intToByte(settingsArray + 11, settings.min_rpm);

    floatToByte(settingsArray + 13, settings.radius, 1000);

    // settingsArray[13] = (byte)settings.pwm_diff;
    // settingsArray[14] = (byte)settings.pwm_zero;

    floatToByte(settingsArray + 15, settings.length, 1000);

    len = 13;
  }
  else if (settingsType == 5)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    intToByte(settingsArray + 5, settings.max_rpm);
    settingsArray[7] = (byte)settings.pwm_zero;
    settingsArray[8] = (byte)settings.pwm_diff;

    floatToByte(settingsArray + 9, settings.angleOff, 100);
    floatToByte(settingsArray + 11, settings.wheelSyncKp, 100);
    len = 13;
  }
  zmcRobotSettingsChar.setValue(settingsArray, 18);
}
