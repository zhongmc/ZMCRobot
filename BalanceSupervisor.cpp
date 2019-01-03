#include "BalanceSupervisor.h"

#include "ZMCRobot.h"

//Robot::Robot(double R, double L, double ticksr, double minRpm, double maxRpm)
BalanceSupervisor::BalanceSupervisor() //: robot(0.0325, 0.155, 390, -120, 140)
{

  m_per_tick = 2 * PI * 0.0325 / 390; //330;
  prev_left_ticks = 0;
  prev_right_ticks = 0;

  m_right_ticks = 0;
  m_left_ticks = 0;
  mSimulateMode = false;
  mIgnoreObstacle = false;

  execTime = 0;

  // start the IMU and filter
  CurieIMU.begin();

  //31.20  -11.70  -3.90 0.67  0.55  -1.04
  //  23.40  -15.60  -3.90 0.67  0.55  -1.10
  //  CurieIMU.setAccelerometerOffset(X_AXIS, 31.20);
  //  CurieIMU.setAccelerometerOffset(Y_AXIS, -11.7);
  //  CurieIMU.setAccelerometerOffset(Z_AXIS, -3.90);
  //  CurieIMU.setGyroOffset(X_AXIS, 0.67);
  //  CurieIMU.setGyroOffset(Y_AXIS, 0.55);
  //  CurieIMU.setGyroOffset(Z_AXIS, -1.04);

  CurieIMU.setGyroRate(GYRO_RATE);
  CurieIMU.setAccelerometerRate(GYRO_RATE);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  //mpu6050.initialize();

  //  accelgyro.initialize();

  filter.begin(GYRO_RATE);
  // filter2.begin(GYRO_RATE);

  // robot.setVel2PwmParam(0.1811, 1.4199, 1.8548); // vel to pwm parameters

  d_stop = 0.05;
  d_at_obs = 0.15;
  d_unsafe = 0.1;
  d_prog = 20;

  m_input.x_g = 1;
  m_input.y_g = 0;
  m_input.v = 0;
  m_input.targetAngle = 0;
  m_input.theta = 0;

  angleType = 0; //use sensor angle
  wheelSyncKP = 0;

  KG_ANG = 0.03; //0.2

  layingDown = false;

  g_fGravityAngle = 0;
  m_estima_angle = 0;

  //  m_vController.setGoal(0, 0, 0);
  speedCounter = 5;
  mVel.vel_l = 0;
  mVel.vel_r = 0;
  pwm_diff = 0; //35
  pwm_zero = 0; //60
  //  mSyncPWM = 0;
  max_pwm = 160;

  mwPWM_L = 0;
  mwPWM_R = 0;

  SETTINGS settings;

  settings.kp = 5;
  settings.ki = 0.0;
  settings.kd = 0.05;
  m_thetaController.updateSettings(settings);

  //  settings.sType = 2;
  //  settings.kp = 30;
  //  settings.ki = 0.0;
  //  settings.kd = 1.0;
  //  m_BalanceController.updateSettings(settings);
  //
  //  settings.sType = 3;
  //  settings.kp = 6;
  //  settings.ki = 0.01;
  //  settings.kd = 0.0;
  //  m_SpeedController.updateSettings(settings);
}

void BalanceSupervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 2 || settings.sType == 0)
    m_BalanceController.updateSettings(settings);

  if (settings.sType == 3)
  {
    m_SpeedController.updateSettings(settings);
  }

  if (settings.sType == 5 || settings.sType == 0)
  {
    settings.max_rpm = 140;
    settings.min_rpm = 0;
    robot.updateSettings(settings);
    pwm_diff = settings.pwm_diff;
    pwm_zero = settings.pwm_zero;

    max_pwm = settings.max_pwm;
    wheelSyncKP = settings.wheelSyncKp;
    d_at_obs = settings.atObstacle;
    d_unsafe = settings.unsafe;

    Serial.print("Update Balance settings:");
    Serial.print(max_pwm);
    Serial.print(",");
    Serial.println(wheelSyncKP);
  }
}

SETTINGS BalanceSupervisor::getSettings(byte settingsType)
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
  settings.wheelSyncKp = wheelSyncKP;
  settings.max_pwm = max_pwm;

  // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: settings for robot; 5: settings for balance robot;
  if (settingsType == 2)
    m_BalanceController.getSettings(&settings);
  else if (settingsType == 3)
  {
    m_SpeedController.getSettings(&settings);
  }

  return settings;
}

void BalanceSupervisor::setGoal(double v, double theta)
{
  m_input.v = v;
  m_input.theta = theta;
  m_input.targetAngle = 0; // 5 * v;

  Serial.print("Set balance car goal to: ");
  Serial.print(v);
  Serial.print(",");
  Serial.println(theta);
  //  m_vController.setGoal(v, theta, robot.theta);
  m_thetaController.setGoal(v, theta, robot.theta);
}

void BalanceSupervisor::stopDrive()
{
  m_input.v = 0;
  m_input.theta = 0;
  m_input.targetAngle = 0;
  mwPWM_L = 0;
  mwPWM_R = 0;
}

void BalanceSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  m_thetaController.setGoal(m_input.v, 0, 0);
  mwPWM_L = 0;
  mwPWM_R = 0;
}

void BalanceSupervisor::reset(long leftTicks, long rightTicks)
{
  d_prog = 20;
  m_BalanceController.reset();
  m_SpeedController.reset();
  prev_left_ticks = leftTicks;
  prev_right_ticks = rightTicks;

  m_right_ticks = 0;
  m_left_ticks = 0;

  robot.reset(leftTicks, rightTicks);
  layingDown = false;
  speedCounter = 0;
  mSpeedPWM = 0;
  //mSyncPWM = 0;

  pwm.pwm_l = 0;
  pwm.pwm_r = 0;

  mwPWM_L = 0;
  mwPWM_R = 0;

  mVel.vel_l = 0;
  mVel.vel_r = 0;
  prev_left_ticks = leftTicks;
  prev_right_ticks = rightTicks;

  g_fGravityAngle = 0;
  m_gyro = 0; //mBalancePWM;

  resetKalman();
}

Position BalanceSupervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void BalanceSupervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
  dis[0] = m_sensor_angle;  // m_sensor_angle;
  dis[1] = g_fGravityAngle; //m_estima_angle;
  dis[2] = m_gyro;          //g_fGravityAngle;
  dis[3] = mBalancePWM;     //mBalancePWM;
  dis[4] = mSpeedPWM;
}

void BalanceSupervisor::execute(long leftTicks, long rightTicks, double dt)
{
  uint32_t timer = micros();

  readIMU();

  double curAngle; // = readIMU();
  //  //m_sensor_angle, m_estima_angle, g_fGravityAngle;
  if (angleType == 0)
    curAngle = m_sensor_angle;
  else if (angleType == 1)
    curAngle = m_estima_angle;
  else
    curAngle = g_fGravityAngle;

  robot.angle = curAngle; //m_sensor_angle; // m_estima_angle m_sensor_angle
  robot.gyro = m_gyro;

  if (!layingDown && (curAngle < -25 || curAngle > 25))
  {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();    //stop motor
  }
  else if (layingDown && (curAngle > -5 && curAngle < 5))
  {
    layingDown = false; // It's no longer laying down
  }

  if (!layingDown)
  {
    m_BalanceController.execute(&robot, &m_input, &m_output, dt);

    mBalancePWM = m_output.w;

    speedCounter++;
    if (speedCounter >= 5)
    {

      if (mSimulateMode)
      {
        robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt * speedCounter);
        //        prev_left_ticks = m_left_ticks;
        //        prev_right_ticks = m_right_ticks;
      }
      else
      {
        robot.updateState(leftTicks, rightTicks, dt * speedCounter);
        //        prev_left_ticks = leftTicks;
        //        prev_right_ticks = rightTicks;
      }

      m_SpeedController.execute(&robot, &m_input, &m_output, speedCounter * dt);
      mSpeedPWM = m_output.w;

      if (m_input.v != 0 || m_input.theta != 0) //转向和直线控制
      {
        m_thetaController.execute(&robot, &m_input, &m_output, speedCounter * dt);

        if (abs(m_output.w) > 0)
        {
          Vel vel = robot.uni_to_diff(0, m_output.w);
          mwPWM_L = robot.vel_l_to_pwm(vel.vel_l);
          mwPWM_R = robot.vel_r_to_pwm(vel.vel_r);

          mwPWM_L = normalize(mwPWM_L, 40);
          mwPWM_R = normalize(mwPWM_R, 40);
        }
        else
        {
          mwPWM_L = 0;
          mwPWM_R = 0;
        }
      }
      else
      {
        mwPWM_L = 0;
        mwPWM_R = 0;
      }
      speedCounter = 0;
    }

    /*
        Serial.print( m_sensor_angle);
        Serial.print(",");
        Serial.print(m_gyro);
        Serial.print(",");
        Serial.print(g_fGravityAngle);
        Serial.print(",");

        Serial.print( mBalancePWM );
        Serial.print(",");
        Serial.print(pwm.pwm_l);
        Serial.print(",");
        Serial.println(pwm.pwm_r);
    */
    double pwm_l, pwm_r;

    mBalancePWM = normalize(mBalancePWM, max_pwm);

    pwm_l = mBalancePWM + mSpeedPWM + mwPWM_L;
    pwm_r = mBalancePWM + mSpeedPWM + mwPWM_R;

    if (pwm_l > 0)
      pwm.pwm_l = pwm_l + pwm_zero;
    else
      pwm.pwm_l = pwm_l - pwm_zero;

    if (pwm_r > 0)
      pwm.pwm_r = pwm_r + pwm_diff + pwm_zero;
    else
      pwm.pwm_r = pwm_r - pwm_diff - pwm_zero;

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

  long ect = micros() - timer;
  if (ect > execTime)
    execTime = ect;

  //  check_states();
}

double BalanceSupervisor::normalize(double in, double limit)
{
  if (in > limit)
    return limit;
  else if (in < -limit)
    return -limit;
  else
    return in;
}

void BalanceSupervisor::check_states()
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

void BalanceSupervisor::resetKalman()
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
  m_estima_angle = Angle_accY;
  //  double dt = (double)1.0 / (double)GYRO_RATE;
}

double BalanceSupervisor::readIMU()
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
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  // print the heading, pitch and roll
  //double roll = filter.getRoll();

  m_sensor_angle = filter.getRoll(); // roll;

  //m_sensor_angle, m_kalman_angle, m_estima_angle,m_filter_angle
  //    pitch = filter.getPitch();
  //    heading = filter.getYaw();

  // double Angle_accY =atan(ay/sqrt(ax*ax + az*az))*180/3.14; //offset
  double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
  double dt = (double)1.0 / (double)GYRO_RATE;

  if ((Angle_accY < -90 && m_estima_angle > 90) || (Angle_accY > 90 && m_estima_angle < -90))
  {
    kalman.setAngle(Angle_accY);
    m_estima_angle = Angle_accY;
  }
  else
    m_estima_angle = kalman.getAngle(Angle_accY, gx, dt); // Calculate the angle using a Kalman filter

  //  kalmanAngle =  kalman.getAngle(Angle_accY, (double)gx, dt);
  // m_estima_angle = 0.98 * (g_fGravityAngle + gx * dt) + 0.02 * Angle_accY; // * RAD_TO_DEG;
  g_fGravityAngle = 0.98 * (g_fGravityAngle + gx * dt) + 0.02 * Angle_accY; //Angle_accY; //0.98 0.02

  //    m_gyro = 0.60*m_gyro + 0.4*m_sensor_angle;

  //    mpu6050.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  //   // convert from raw data to gravity and degrees/second units
  //    ax = convertRawAcceleration(aix);
  //    ay = convertRawAcceleration(aiy);
  //    az = convertRawAcceleration(aiz);
  //    gx = convertRawGyro(gix);
  //    gy = convertRawGyro(giy);
  //    gz = convertRawGyro(giz);
  //
  //    filter2.updateIMU(gx, gy, gz, ax, ay, az);
  //    m_estima_angle = filter.getRoll(); // roll;

  //  Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
  // g_fGravityAngle = 0.98 * (g_fGravityAngle + gx * dt) + 0.02 * Angle_accY; //0.98 0.02

  return m_estima_angle; //g_fGravityAngle;// m_sensor_angle;
}

double BalanceSupervisor::convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  double a = (aRaw * 2.0) / 32768.0;
  return a;
}

double BalanceSupervisor::convertRawGyro(int gRaw)
{
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  double g = (gRaw * 250.0) / 32768.0;
  return g;
}

//------------------------------------------------------------------
double BalanceSupervisor::estima_cal(double estima, double metron, double KG)
{
  double result1 = estima + KG * (metron - estima);
  return result1;
}
