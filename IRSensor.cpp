#include "IRSensor.h"

//fallow wall 时给出的无障碍物时的坐标距离
#define WALL_DIS 0.35

/*
static double ab[10][2] = {
{-0.00015346, 0.207142857},
{-0.000247897, 0.273076923},
{-0.000537109, 0.416666667},
{-0.000805664, 0.525},
{-0.001074219, 0.616666667},
{-0.001074219, 0.616666667},
{-0.002148438, 0.883333333},
{-0.003222656, 1.1},
{-0.00604248, 1.5375},
{-0.008056641, 1.8}};

static int xval[10] = {698,496,403,341,295,248,202,155,130,124 };

*/

IRSensor::IRSensor()
{
  mSensorType = GP2Y0A21;
  setDistance(getMaxDistance());
}

IRSensor::IRSensor(SENSOR_TYPE sensorType)
{
  mSensorType = sensorType;
  setDistance(getMaxDistance());
}

IRSensor::IRSensor(double xs, double ys, double thetas, byte pin, SENSOR_TYPE sensorType)
{
  x_s = xs;
  y_s = ys;
  anologPin = pin;
  theta_s = thetas;
  cos_theta = cos(thetas);
  sin_theta = sin(thetas);
  mSensorType = sensorType;
  setDistance(getMaxDistance());
}

void IRSensor::SetSensorType(SENSOR_TYPE sensorType)
{
  mSensorType = sensorType;
  setDistance(getMaxDistance());
}

double IRSensor::readDistance(byte pin)
{
  int digVal = analogRead(pin);
  return getDistance(digVal);
}

void IRSensor::readPosition(double xs, double ys, double thetas, byte pin)
{
  x_s = xs;
  y_s = ys;
  anologPin = pin;
  theta_s = thetas;
  cos_theta = cos(thetas);
  sin_theta = sin(thetas);
  readPosition();
}

void IRSensor::readPosition()
{
  int digVal = analogRead(anologPin);
  distance = getDistance(digVal);
  x = x_s + distance * cos_theta;
  y = y_s + distance * sin_theta;
}

void IRSensor::setDistance(double dis)
{
  distance = dis;
  x = x_s + distance * cos_theta;
  y = y_s + distance * sin_theta;
}

void IRSensor::applyGeometry(double xc, double yc, double sinTheta, double cosTheta)
{
  xw = xc + x * cosTheta - y * sinTheta;
  yw = yc + x * sinTheta + y * cosTheta;
}

 Vector IRSensor::getWallVector(double xc, double yc, double theta, double d)
 {
   Vector p;
   p.x = xw;
   p.y = yw;
   if( distance > d )
   {
     double dis = distance;
     setDistance(d);
     applyGeometry(xc, yc, sin(theta), cos(theta));
     p.x = xw;
     p.y = yw;

     setDistance(dis);
     applyGeometry(xc, yc, sin(theta), cos(theta));
   }
   return p;
 }

static double gp2y0a21[4][4] = {
    {233, 2.00E-05, -0.0109, 1.8439},
    {334, 3.00E-06, -0.0033, 0.9942},
    {508, -4.00E-07, -0.0002, 0.3666},
    {853, 2.00E-07, -0.0005, 0.3361}};

double gp2y0a41[4][4] = {
    {124, -2.00E-05, 0.0015, 0.4689},
    {334, 3.00E-06, -0.0022, 0.5273},
    {543, 5.00E-07, -0.0007, 0.2979},
    {853, 7.00E-08, -0.0002, 0.1564}};

double IRSensor::getMaxDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.8;
  else if (mSensorType == GP2Y0A41)
    return 0.3;
  return 1;
}
double IRSensor::getMinDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.1;
  else if (mSensorType == GP2Y0A41)
    return 0.04;
  return 0.1;
}

double IRSensor::getDistance(int digitalVal)
{
  int idx = 0;
  double *p;
  int len;

  if (mSensorType == GP2Y0A21)
  {
    if (digitalVal < 124)
      return 0.8;
    else if (digitalVal > 927)
      return 0.1;

    p = (double *)gp2y0a21;
    len = 3;
  }
  else
  {
    if (digitalVal < 96)
      return 0.3;
    if (digitalVal > 853)
      return 0.04;

    p = (double *)gp2y0a41;
    len = 3;
  }

  while (idx < len)
  {
    if (digitalVal <= *(p + idx * 4))
      break;
    idx++;
  }

  double val = *(p + idx * 4 + 1) * sq(digitalVal) + *(p + idx * 4 + 2) * digitalVal + *(p + idx * 4 + 3);
  if (val < 0.04)
    val = 0.04;
  else if (val > 0.8)
    val = 0.8;
  return val;
  /* 
  while(idx < 9 )
  {
    if( digitalVal >= xval[idx] )
      break;
     idx++;
   }
   return ab[idx][0]*digitalVal + ab[idx][1];  //y = ax + b
*/
}
