#include "Robot.h"
#include "ZMCRobot.h"

//#define PI 3.141592653589793

extern double ultrasonicDistance;

Robot::Robot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  init(0.065 / 2, 0.125, 20, 20, 40, 200, GP2Y0A41);
}

Robot::Robot(double R, double L, double ticksr_l, double ticksr_r, double minRpm, double maxRpm)
{
  init(R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
}

void Robot::init(double R, double L, double ticksr_l, double ticksr_r, double minRpm, double maxRpm, SENSOR_TYPE sensorType)
{
  x = 0;
  y = 0;
  theta = 0;
  //  vel_l = 0;
  //  vel_r = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;

  wheel_radius = R;           //0.065 / 2;
  wheel_base_length = L;      // 0.127;
  ticks_per_rev_l = ticksr_l; //20;
  ticks_per_rev_r = ticksr_r; //20;
  m_per_tick_l = 2 * PI * wheel_radius / ticks_per_rev_l;
  m_per_tick_r = 2 * PI * wheel_radius / ticks_per_rev_r;

  max_rpm = maxRpm; //160; //267
  max_vel = max_rpm * 2 * PI / 60;

  min_rpm = minRpm; // 70; //113
  min_vel = min_rpm * 2 * PI / 60;

  max_v = max_vel * wheel_radius;
  min_v = min_vel * wheel_radius;
  max_w = (wheel_radius / wheel_base_length) * (max_vel); // - min_vel);
  min_w = (wheel_radius / wheel_base_length) * (min_vel);

  pwm_diff = 0;
  angleOff = 0;

  irSensors[0] = new IRSensor(-0.1, 0.055, PI / 2, A1, sensorType); //A1
  irSensors[1] = new IRSensor(0.075, 0.06, PI / 4, A2, sensorType);
  irSensors[2] = new IRSensor(0.085, 0., 0, A3, sensorType);
  irSensors[3] = new IRSensor(0.075, -0.06, -PI / 4, A4, sensorType);
  irSensors[4] = new IRSensor(-0.1, -0.055, -PI / 2, A5, sensorType);

  haveIrSensor[0] = true;
  haveIrSensor[1] = true;
  haveIrSensor[2] = true;
  haveIrSensor[3] = true;
  haveIrSensor[4] = true;
}

void Robot::setIRSensorType(SENSOR_TYPE sensorType)
{
  for (int i = 0; i < 5; i++)
    irSensors[i]->SetSensorType(sensorType);
}

void Robot::updateSettings(SETTINGS settings)
{
  max_rpm = settings.max_rpm; //267
  max_vel = max_rpm * 2 * PI / 60;

  min_rpm = settings.min_rpm; //113
  min_vel = min_rpm * 2 * PI / 60;

  max_v = max_vel * wheel_radius;
  min_v = min_vel * wheel_radius;
  max_w = (wheel_radius / wheel_base_length) * (max_vel - min_vel);
  min_w = (wheel_radius / wheel_base_length) * (2 * min_vel);
  pwm_diff = settings.pwm_diff;
  angleOff = settings.angleOff;
}

void Robot::reset(long left_ticks, long right_ticks)
{
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;
  //  x = 0;
  //  y = 0;
  //  theta = 0;
}

void Robot::updateState(long left_ticks, long right_ticks, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    readIRSensors();
    velocity = 0;
    return; //no change
  }

  double d_right, d_left, d_center;

  d_left = (left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (right_ticks - prev_right_ticks) * m_per_tick_r;

  prev_right_ticks = right_ticks;
  prev_left_ticks = left_ticks;

  d_center = (d_right + d_left) / 2;
  velocity = d_center / dt;

  double phi = (d_right - d_left) / wheel_base_length;

  x = x + d_center * cos(theta);
  y = y + d_center * sin(theta);
  theta = theta + phi;
  theta = atan2(sin(theta), cos(theta));

  readIRSensors();
}

void Robot::readIRSensors()
{
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);

  for (int i = 0; i < 5; i++)
  {
    if (haveIrSensor[i])
      irSensors[i]->readPosition();
  }
  if (ultrasonicDistance < MAX_ULTRASONIC_DIS)
  {
    irSensors[2]->setDistance(ultrasonicDistance);
  }
  for (int i = 0; i < 5; i++)
    irSensors[i]->applyGeometry(x, y, sinTheta, cosTheta);
}

void Robot::getRobotInfo()
{
  Serial.print("x:");
  Serial.print(x);
  Serial.print(",y:");
  Serial.print(y);

  Serial.print(",theta:");
  Serial.print(theta);

  Serial.print(",v:");
  Serial.println(10 * velocity);

  Serial.print("max_vel:");
  Serial.print(max_vel);
  Serial.print(", min_vel:");
  Serial.print(min_vel);

  Serial.print(",max_rpm:");
  Serial.print(max_rpm);
  Serial.print(", min_rpm:");
  Serial.print(min_rpm);
  Serial.print(", max_w:");
  Serial.print(max_w);
  Serial.print(", min_w:");
  Serial.println(min_w);

  /*
  double pwm_min_l = vel_l_to_pwm(min_vel);
  double pwm_min_r = vel_r_to_pwm(min_vel);
  double pwm_max_l = vel_l_to_pwm(max_vel);
  double pwm_max_r = vel_r_to_pwm(max_vel);

  Serial.print("PWM:(min,max)-l,(min,max)-r:");
  Serial.print(pwm_min_l);
  Serial.print(",");
  Serial.print(pwm_max_l);
  Serial.print(",");
  Serial.print(pwm_min_r);
  Serial.print(",");
  Serial.println(pwm_max_r);
*/

  Serial.print("Robot param(R,L, tks/r):");
  Serial.print(100 * wheel_radius);
  Serial.print(",");
  Serial.print(100 * wheel_base_length);
  Serial.print(",");
  Serial.print(ticks_per_rev_l);
  Serial.print(",");
  Serial.println(ticks_per_rev_r);

  Serial.print("Balance ang=");
  Serial.print(angle);
  Serial.print(", gyro=");
  Serial.println(gyro);
  if (irSensors[0]->getSensorType() == GP2Y0A41) //GP2Y0A41 = 0,     //4-30cm  GP2Y0A21
    Serial.println("IR GP2Y0A41 d:");
  else
    Serial.println("IR GP2Y0A21 d:");

  for (int i = 0; i < 5; i++)
  {
    if (haveIrSensor[i])
      irSensors[i]->readPosition();
    Serial.print(irSensors[i]->distance);
    Serial.print(",");
  }
  Serial.println(";");
}

IRSensor **Robot::getIRSensors()
{
  return irSensors;
}

//get the most closer obstacle distance of the front 3;
double Robot::getObstacleDistance()
{

  return irSensors[2]->distance;

  double d = irSensors[1]->distance;
  if (d > irSensors[2]->distance)
    d = irSensors[2]->distance;
  if (d > irSensors[3]->distance)
    d = irSensors[3]->distance;
  return d;
}

Vel Robot::uni_to_diff(double v, double w)
{
  Vel vel;
  vel.vel_r = (2 * v + w * wheel_base_length) / (2 * wheel_radius);
  vel.vel_l = (2 * v - w * wheel_base_length) / (2 * wheel_radius);
  return vel;

  //double vv = -0.0537*w + 0.8054;
  //if( vv < 0.1)
  //  vv = 0.1;
  // if( vv > 0.8 )
  //  vv = 0.8;
  //
  //  vel.vel_r = (2 * vv + w * wheel_base_length) / (2 * wheel_radius);
  //  vel.vel_l = (2 * vv - w * wheel_base_length) / (2 * wheel_radius);
  //
  //  return vel;
}

Output Robot::diff_to_uni(double vel_l, double vel_r)
{
  Output out;
  if (vel_l + vel_r == 0)
  {
    Serial.println("div by o...in robot 1");
    out.v = 0.5;
    return out;
  }
  else
    out.v = wheel_radius / 2 * (vel_l + vel_r);

  if (vel_r - vel_l == 0)
  {
    Serial.println("div by o...in robot 2");
    out.w = PI / 2;
  }
  else
    out.w = wheel_radius / wheel_base_length * (vel_r - vel_l);

  return out;
}
