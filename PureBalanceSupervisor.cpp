#include "PureBalanceSupervisor.h"

#include "ZMCRobot.h"

PureBalanceSupervisor::PureBalanceSupervisor()
{

  // m_per_tick = 2 * PI * 0.0325 / 390; //330;
  prev_left_ticks = 0;
  prev_right_ticks = 0;

  m_right_ticks = 0;
  m_left_ticks = 0;
  mSimulateMode = false;
  mIgnoreObstacle = false;

  execTime = 0;

  d_stop = 0.05;
  d_at_obs = 0.15;
  d_unsafe = 0.1;
  d_prog = 20;

  m_input.x_g = 1;
  m_input.y_g = 0;
  m_input.v = 0;
  m_input.targetAngle = 0;
  m_input.theta = 0;

  mSpeedLoop = false;
  mThetaLoop = false;
  mSendIMUInfo = false;

  // angleType = 0; //use sensor angle
  KG_ANG = 0.03; //0.2
  layingDown = false;
  //  m_vController.setGoal(0, 0, 0);
  speedCounter = 5;
  mVel.vel_l = 0;
  mVel.vel_r = 0;
  pwm_diff = 10; //35
  pwm_zero = 50; //60
  //  mSyncPWM = 0;
  max_pwm = 240;

  mwPWM_L = 0;
  mwPWM_R = 0;

  b_kp = 38;
  b_kd = 0.58;

  s_kp = 5;
  s_ki = 0.01;
  sIntegral = 0;

  t_kp = 5;

  KG = 0.05;
  m_x_angle = 0;
  keepTheta = false;

  curW = 0;
  mW = 0;
  m_state = 0; //0 normal 1 drive 2 goto-goal
}

void PureBalanceSupervisor::init()
{
  // start the IMU and filter
  CurieIMU.begin();

  //公司板 com3
  //144.3， 19.5， -89.7， -0.79, 1.04， -0.12

  //家 com5
  //35.10	-27.30	42.90	0.85	-0.12	-0.55

  //  com3
  // CurieIMU.setAccelerometerOffset(X_AXIS, 144.3);
  // CurieIMU.setAccelerometerOffset(Y_AXIS, 19.5);
  // CurieIMU.setAccelerometerOffset(Z_AXIS, -89.7);
  // CurieIMU.setGyroOffset(X_AXIS, -0.79);
  // CurieIMU.setGyroOffset(Y_AXIS, 1.04);
  // CurieIMU.setGyroOffset(Z_AXIS, -0.12);

  //  com5
  CurieIMU.setAccelerometerOffset(X_AXIS, 35.10);
  CurieIMU.setAccelerometerOffset(Y_AXIS, -27.3);
  CurieIMU.setAccelerometerOffset(Z_AXIS, 42.9);
  CurieIMU.setGyroOffset(X_AXIS, 0.85);
  CurieIMU.setGyroOffset(Y_AXIS, -0.12);
  CurieIMU.setGyroOffset(Z_AXIS, -0.55);

  CurieIMU.setGyroRate(GYRO_RATE);
  CurieIMU.setAccelerometerRate(GYRO_RATE);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  // filter.begin(GYRO_RATE);

  //mpu6050.initialize();
  //  accelgyro.initialize();
  // filter2.begin(GYRO_RATE);
  // robot.setVel2PwmParam(0.1811, 1.4199, 1.8548); // vel to pwm parameters
}

void PureBalanceSupervisor::setSettings(SETTINGS settings)
{
  settings.max_rpm = 140;
  settings.min_rpm = 0;
  // robot.updateSettings(settings);
  pwm_diff = settings.pwm_diff;
  pwm_zero = settings.pwm_zero;

  max_pwm = settings.max_pwm;
  d_at_obs = settings.atObstacle;
  d_unsafe = settings.unsafe;
}

SETTINGS PureBalanceSupervisor::getSettings()
{
  SETTINGS settings;

  settings.sType = 0; //settingsType;

  settings.atObstacle = d_at_obs;
  settings.unsafe = d_unsafe;
  settings.dfw = d_fw;
  settings.velocity = m_input.v;
  settings.max_rpm = robot.max_rpm;
  settings.min_rpm = robot.min_rpm;

  settings.pwm_diff = pwm_diff;
  settings.pwm_zero = pwm_zero;
  settings.angleOff = robot.angleOff;
  settings.max_pwm = max_pwm;
  return settings;
}

PIDParam PureBalanceSupervisor::getBalancePIDParam()
{
  PIDParam pid;
  pid.kp = b_kp;
  pid.ki = 0;
  pid.kd = b_kd;

  return pid;
}

PIDParam PureBalanceSupervisor::getSpeedPIDParam()
{
  PIDParam pid;
  pid.kp = s_kp;
  pid.ki = s_ki;
  pid.kd = 0;
  return pid;
}

PIDParam PureBalanceSupervisor::getThetaPIDParam()
{
  PIDParam pid;
  pid.kp = t_kp;
  pid.ki = 0;
  pid.kd = 0;
  return pid;
}

void PureBalanceSupervisor::setBalancePIDParam(PIDParam pid)
{
  b_kp = pid.kp;
  b_kd = pid.kd;
}
void PureBalanceSupervisor::setSpeedPIDParam(PIDParam pid)
{
  s_kp = pid.kp;
  s_ki = pid.ki;
}

void PureBalanceSupervisor::setThetaPIDParam(PIDParam pid)
{
  t_kp = pid.kp;
}

void PureBalanceSupervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 2 || settings.sType == 0)
  {
    b_kp = settings.kp;
    b_kd = settings.kd;
  }
  if (settings.sType == 3)
  {
    s_kp = settings.kp;
    s_ki = settings.ki;
    // m_SpeedController.updateSettings(settings);
    // m_thetaController.updateSettings(settings);
  }

  if (settings.sType == 5 || settings.sType == 0)
  {
    settings.max_rpm = 140;
    settings.min_rpm = 0;
    robot.updateSettings(settings);
    pwm_diff = settings.pwm_diff;
    pwm_zero = settings.pwm_zero;

    max_pwm = settings.max_pwm;
    d_at_obs = settings.atObstacle;
    d_unsafe = settings.unsafe;
  }
}

SETTINGS PureBalanceSupervisor::getSettings(byte settingsType)
{
  SETTINGS settings;

  settings.sType = settingsType;

  settings.atObstacle = d_at_obs;
  settings.unsafe = d_unsafe;
  settings.dfw = d_fw;
  settings.velocity = m_input.v;
  settings.max_rpm = robot.max_rpm;
  settings.min_rpm = robot.min_rpm;

  settings.pwm_diff = pwm_diff;
  settings.pwm_zero = pwm_zero;
  settings.angleOff = robot.angleOff;
  settings.max_pwm = max_pwm;
  settings.length = robot.wheel_base_length;
  settings.radius = robot.wheel_radius;

  // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: settings for robot; 5: settings for balance robot;
  if (settingsType == 2)
  {
    settings.kp = b_kp;
    settings.kd = b_kd;
    settings.ki = 0;
  }
  else if (settingsType == 3)
  {
    settings.kp = s_kp;
    settings.ki = s_ki;
    settings.kd = 0;
  }
  else if (settingsType == 4)
  {
    settings.kp = t_kp;
    settings.ki = 0;
    settings.kd = 0;
  }

  return settings;
}

void PureBalanceSupervisor::startGotoGoal()
{
  m_state = 2;
}

//move robot to target x,y
void PureBalanceSupervisor::setGotoGoal(double x, double y, double theta)
{
  m_Goal.x = x;
  m_Goal.y = y;

  m_input.x_g = x;
  m_input.y_g = y;
}

//drive the robot
void PureBalanceSupervisor::setGoal(double v, double w)
{
  m_input.v = v;
  m_input.theta = w;
  m_input.targetAngle = 0; // 5 * v;

  if (w == 0 && curW != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    keepTheta = true;
    thetaPrevMillis = millis();

    //    mTheta = robot.theta; //转弯结束，保留当前角度
  }
  curW = w;
  mW = w;
  if (mW == 0)
  {
    Serial.println("zero mw!");
  }

  Serial.print("Set balance car goal to: ");
  Serial.print(v);
  Serial.print(",");
  Serial.println(w);
  //  m_vController.setGoal(v, theta, robot.theta);
  // m_thetaController.setGoal(v, theta, robot.theta);
}

void PureBalanceSupervisor::stopDrive()
{
  m_input.v = 0;
  m_input.theta = 0;
  m_input.targetAngle = 0;
  mwPWM_L = 0;
  mwPWM_R = 0;
  keepTheta = false;
  m_state = 0;
  curW = 0;
  mW = 0;
}

void PureBalanceSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  // m_thetaController.setGoal(m_input.v, 0, 0);
  mwPWM_L = 0;
  mwPWM_R = 0;
  sIntegral = 0;
  keepTheta = false;
  curW = 0;
  mW = 0;
}

void PureBalanceSupervisor::reset(long leftTicks, long rightTicks)
{
  d_prog = 20;
  // m_BalanceController.reset();
  // m_SpeedController.reset();
  prev_left_ticks = leftTicks;
  prev_right_ticks = rightTicks;

  m_right_ticks = 0;
  m_left_ticks = 0;

  robot.reset(leftTicks, rightTicks);
  layingDown = false;
  hangUp = false;

  speedCounter = 0;
  mSpeedPWM = 0;
  mThetaPWM = 0;

  pwm.pwm_l = 0;
  pwm.pwm_r = 0;

  mwPWM_L = 0;
  mwPWM_R = 0;

  mVel.vel_l = 0;
  mVel.vel_r = 0;
  prev_left_ticks = leftTicks;
  prev_right_ticks = rightTicks;

  // m_estima_angle = 0;
  m_gyro = 0; //mBalancePWM;
  sIntegral = 0;

  keepTheta = false;

  curW = 0;
  mW = 0;

  resetKalman();
}

Position PureBalanceSupervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void PureBalanceSupervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
  dis[0] = m_sensor_angle; // m_sensor_angle;
  dis[1] = m_kalman_angle; //m_estima_angle;
  dis[2] = m_gyro;         //g_fGravityAngle;
  dis[3] = mBalancePWM;    //mBalancePWM;
  dis[4] = mSpeedPWM;
}

// void balanceOut(double dt);
// void speedOut(double dt);
// void thetaOut(double targetTheta, double robotTheta, double dt);

void PureBalanceSupervisor::setBeSendIMUInfo(bool val)
{
  mSendIMUInfo = val;
}

void PureBalanceSupervisor::setBeSpeedLoop(bool val)
{
  mSpeedPWM = 0;
  mSpeedLoop = val;
}

void PureBalanceSupervisor::setBeThetaLoop(bool val)
{
  mThetaPWM = 0;
  mThetaLoop = val;
}

void PureBalanceSupervisor::balanceOut(double dt)
{
  float e_k;
  /* Update PID values */
  e_k = m_input.targetAngle - robot.angle - robot.angleOff; //roll;  // pitch;
  mBalancePWM = b_kp * e_k - b_kd * robot.gyro;
}

void PureBalanceSupervisor::speedOut(long leftTicks, long rightTicks, double dt)
{
  // double lt = leftTicks - prev_left_ticks;
  // double rt = rightTicks - prev_right_ticks;
  // prev_left_ticks = leftTicks;
  // prev_right_ticks = rightTicks;

  // lt = lt / dt;
  // rt = rt / dt;
  // double speed = (lt / robot.ticks_per_rev_l + rt / robot.ticks_per_rev_r) / 2.0;
  double e = m_input.v - robot.velocity;
  sIntegral = sIntegral + s_ki * e;
  mSpeedPWM = s_kp * e + sIntegral;
}

void PureBalanceSupervisor::thetaOut(double dt)
{

  if (mW != 0) //turning
  {
    mThetaPWM = t_kp * mW;
    return;
  }

  if (keepTheta == true)
  {
    if (millis() - thetaPrevMillis > 120)
    {
      keepTheta = false;
      okToKeep = true;
    }
    else
    {
      mThetaPWM = 0;
      return;
    }
  }

  if (okToKeep)
  {
    okToKeep = false;
    m_input.theta = robot.theta;
  }
  double e = m_input.theta - robot.theta;
  mThetaPWM = t_kp * e;
}

void PureBalanceSupervisor::execute(long leftTicks, long rightTicks, double dt)
{

  long startTime = micros();

  readIMU(dt);

  if (mSendIMUInfo)
    sendIMUInfo();

  robot.angle = m_kalman_angle; //m_sensor_angle; // m_estima_angle m_sensor_angle
  robot.gyro = m_gyro;

  if (!layingDown && (m_kalman_angle < -25 || m_kalman_angle > 25))
  {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();    //stop motor
    return;
  }
  else if (layingDown && (m_kalman_angle > -5 && m_kalman_angle < 5))
  {
    layingDown = false; // It's no longer laying down
  }

  if (!hangUp)
  {
    if (abs(m_x_angle) > 10)
    {
      hangUp = true;
      stopAndReset();
      Serial.print(m_x_angle);
      Serial.println(", hang up...");
      return;
    }
  }
  else
  {
    if (abs(m_x_angle) < 5)
    {
      hangUp = false;
      Serial.print(m_x_angle);
      Serial.println(", recover from hang up ! ");
    }
  }

  if (!layingDown && !hangUp)
  {

    balanceOut(dt);

    speedCounter++;
    if (speedCounter >= 10)
    {
      if (mSimulateMode)
      {
        robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt * speedCounter);
      }
      else
      {
        robot.updateState(leftTicks, rightTicks, dt * speedCounter);
      }
      speedOut(leftTicks, rightTicks, dt * speedCounter);
      thetaOut(dt * speedCounter);
      speedCounter = 0;
    }
  }

  double pwm_l, pwm_r;

  // mBalancePWM = normalize(mBalancePWM, max_pwm);

  pwm_l = mBalancePWM;
  pwm_r = mBalancePWM;

  if (mSpeedLoop)
  {
    pwm_l = pwm_l + mSpeedPWM;
    pwm_r = pwm_r + mSpeedPWM;
  }

  if (mThetaLoop)
  {
    pwm_l = pwm_l + mThetaPWM;
    pwm_r = pwm_r - mThetaPWM;
  }

  if (pwm_l > 0)
    pwm.pwm_l = pwm_l + pwm_zero;
  else
    pwm.pwm_l = pwm_l - pwm_zero;

  if (pwm_r > 0)
    pwm.pwm_r = pwm_r + pwm_zero; //+pwm_diff
  else
    pwm.pwm_r = pwm_r - pwm_zero; //-pwm_diff

  pwm_l = normalize(pwm_l, max_pwm);
  pwm_r = normalize(pwm_r, max_pwm);

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

#ifdef _DEBUG_
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

  long ect = micros() - startTime;
  if (ect > execTime)
    execTime = ect;

  //  check_states();
}

void PureBalanceSupervisor::sendIMUInfo()
{
  String retStr;

  char tmp[10];
  itoa((int)(10000 * m_sensor_angle), tmp, 10);
  String sa = tmp;

  itoa((int)(10000 * m_gyro), tmp, 10);
  String sg = tmp;

  itoa((int)(10000 * m_kalman_angle), tmp, 10);
  String ka = tmp;

  // itoa((int)(10000 * m_kalman_gyro), tmp, 10);
  // String kg = tmp;
  // itoa((int)(10000 * m_x_angle), tmp, 10);
  // String ma = tmp;
  // itoa((int)(10000 * m_km_angle), tmp, 10);
  // String kma = tmp;

  retStr = "MU" + sa + "," + sg + "," + ka; //  + "," + kg + "," + kma;
  Serial.println(retStr);
  // Serial.print("MU");
  // Serial.print((int)(10000 * m_sensor_angle));
  // Serial.print(",");
  // Serial.print((int)(10000 * m_gyro));
  // Serial.print(",");
  // Serial.print((int)(10000 * m_kalman_angle));
  // Serial.print(",");
  // Serial.print((int)(10000 * m_kalman_gyro));
  // Serial.print(",");
  // Serial.println((int)(10000 * m_madgwick_angle));
  // Serial.print(",");
  // Serial.println((int)(10000 * m_km_angle));
}

double PureBalanceSupervisor::normalize(double in, double limit)
{
  if (in > limit)
    return limit;
  else if (in < -limit)
    return -limit;
  else
    return in;
}

void PureBalanceSupervisor::check_states()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));
  if (d < d_prog - 0.1)
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
  for (int i = 0; i < 5; i++)
  {
    if (irSensors[i]->distance < d_at_obs)
      at_obstacle = true;
    if (irSensors[i]->distance < d_unsafe)
      unsafe = true;
  }
}

void PureBalanceSupervisor::resetKalman()
{

  int aix, aiy, aiz;
  int gix, giy, giz;
  double ax, ay, az;
  double gx, gy, gz;

  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
  kalman.setAngle(Angle_accY);
  // km.setAngle(Angle_accY);
  // m_estima_angle = Angle_accY;
  //  double dt = (double)1.0 / (double)GYRO_RATE;
}

void PureBalanceSupervisor::readIMU(double dt)
{
  int aix, aiy, aiz;
  int gix, giy, giz;
  double ax, ay, az;
  double gx, gy, gz;

  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  m_gyro = gx; //

  // update the filter, which computes orientation
  // filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  //    double roll = filter.getRoll();
  //    pitch = filter.getPitch();
  //    heading = filter.getYaw();

  double Angle_accY = atan(ay / sqrt(ax * ax + az * az)) * 180 / 3.14; //offset

  m_sensor_angle = Angle_accY; //filter.getRoll();

  // if ((Angle_accY < -90 && m_estima_angle > 90) || (Angle_accY > 90 && m_estima_angle < -90))
  // {
  //   kalman.setAngle(Angle_accY);
  //   m_kalman_angle = Angle_accY;
  //   m_kalman_gyro = m_gyro;
  // }
  // else
  {
    m_kalman_angle = kalman.getAngle(m_sensor_angle, gx, dt); // Calculate the angle using a Kalman filter
    m_kalman_gyro = kalman.getRate();
  }
  // double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
  // m_km_angle = km.getAngle(Angle_accY, gx, dt);

  double angle_accX = atan2((double)ax, (double)az) * RAD_TO_DEG;
  m_x_angle = estima_cal(m_x_angle, angle_accX, gy, dt, 0.05);
}

double PureBalanceSupervisor::convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  double a = (aRaw * 2.0) / 32768.0;
  return a;
}

double PureBalanceSupervisor::convertRawGyro(int gRaw)
{
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  double g = (gRaw * 250.0) / 32768.0;
  return g;
}

//一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
// angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
double PureBalanceSupervisor::estima_cal(double angle, double g_angle, double gyro, double dt, double KG)
{
  double result = KG * g_angle + (1 - KG) * (angle + gyro * dt);
  return result;
}
