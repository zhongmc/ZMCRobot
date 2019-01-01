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
static int comDataCount;

//extern Supervisor supervisor;
extern double gp2y0a41[3][4];
extern long count1,count2;

extern bool openDebug;

extern double batteryVoltage;

void checkSerialData()
{

  //read speed setting from serial
  if ( Serial.available() )
  {
    while (Serial.available() > 0)
    {
      char ch = Serial.read();
      comData[comDataCount++] = ch;
      if ( ch == ';') //new command
      {
        processCommand(comData, comDataCount);
        comDataCount = 0;
      }
      if ( comDataCount > 20) //some error
      {
        Serial.print("UnKnow Command:");
        comData[comDataCount] = 0;
        Serial.println( comData );
        comDataCount = 0;
      }
    }
  }
}


void printCmds()
{
  return;
  
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
//  Serial.println("***** com commands ***********");
//  Serial.println(" ?; print this info");
//  Serial.println(" gr[]; controller infos");
//  Serial.println(" gs; settings");
//  Serial.println(" sb; start balance");
//  Serial.println(" st; stop robot");
//  Serial.println(" gd; distance of ultraSonic");
//  Serial.println(" bp[]; balance kp");
//  Serial.println(" bi[]; balance ki");
//  Serial.println(" bd[]; balance kd");
//
//  Serial.println(" vp[]; velocity kp");
//  Serial.println(" vi[]; velocity ki");
//  Serial.println(" vd[]; velocity kd");
//  Serial.println(" od; open debug");
//  Serial.println(" cd; close debug");
//  Serial.println(" ci; calibrate imu");
  

}

void processCommand(char* buffer, int bufferLen)
{
  if( tolower(buffer[0]) == '?')
  {
    printCmds();
    return;
  }
  if ( bufferLen <= 2)
    return;
  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);
  if ( ch0 == 'm' && ch1 == 'u')
  {
    printMenu();
    return;
  }
  else if ( ch0 == 'g' && ch1 == 'i')
  {
    sendTestInfo();
    return;
  }
  else if ( ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n\r\n=========================================================");
    Serial.print("Voltage:");
    Serial.print(batteryVoltage);
    Serial.print(", ultrasonic dis:");
    Serial.println(ultrasonicDistance);
#if CAR_TYPE == DRIVE_CAR    
    Serial.println("\r\n=========================================================");
    supervisor.getRobotInfo();
    Serial.println("\r\n=========================================================");
    driveSupervisor.getRobotInfo();
#else
    Serial.println("\r\n=========================================================");
    balanceSupervisor.getRobotInfo();
#endif

  }

  else if( ch0=='o' && ch1 == 'd') //open debug
  {
    openDebug = true;
  }
  else if( ch0 == 'c' && ch1 == 'd') //close debug
  {
    openDebug = false;
  }
  else if ( ch0 == 'g' && ch1 == 's')
  {
    sendSettings();
  }
  else if ( ch0 == 'g' && ch1 == 'd')
  {
    Serial.print("Distance of ultraSonic:");
    if ( echoTime > 0 )
    {
      float dist = 0.017 * echoTime;
      //     float dist = 17000/echoTime;
      Serial.println(dist);
    }
    else
    {
      Serial.println("NA");
    }
  }
  else if ( ch0 == 's' && ch1 == 't') //stop
  {
    Serial.println("Stop!");
    stopRobot();
  }
  else if ( ch0 == 's' && ch1 == 'b') //start balance
  {
    Serial.println("Start balance!");
#if CAR_TYPE == BALANCE_CAR    
    startBalance();
#endif
  }
  else if(ch0 == 'c' && ch1 == 'i') //count info
  {
    Serial.print("C1=");
    Serial.print(count1);
    Serial.print(", C2=");
    Serial.println(count2);  
  }
else if( ch0 == 'm' && ch1 == 'm') // move motor
{
  int pwm = atoi( buffer + 2);
  Serial.print( "Move motor: ");
  Serial.println( pwm );
  
    Serial.print("C1=");
    Serial.print(count1);
    Serial.print(", C2=");
    Serial.println(count2);  
    Serial.print("current time:");
    Serial.println(millis());
    MoveMotor( pwm );
  
}
  
#if CAR_TYPE == BALANCE_CAR
  else if ( ch0 == 'b') //setting  balance values
  {
    double val;
    if ( ch1 == 'p' )
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KP to:");
      Serial.println(val);

      balanceSupervisor.setBalanceCtrlParam(val, 0);



    }
    else if ( ch1 == 'i' )
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KI to:");
      Serial.println(val);
      balanceSupervisor.setBalanceCtrlParam(val, 1);
    }
    else if ( ch1 == 'd')
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set balance KD to:");
      Serial.println(val);
      balanceSupervisor.setBalanceCtrlParam(val, 2);

    }
 }
   
   else if( ch0 == 'c' && ch1 == 'i')
    {
    //  CalibrateIMU();
    }


  else if ( ch0 == 'v') //setting  speed values
  {
    double val;
    if ( ch1 == 'p' )
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set speed KP to:");
      Serial.println(val);

      balanceSupervisor.setSpeedCtrlParam(val, 0);



    }
    else if ( ch1 == 'i' )
    {
      buffer[bufferLen - 1] = 0;
      val = atof(buffer + 2);
      Serial.print("Set Speed KI to:");
      Serial.println(val);
      balanceSupervisor.setSpeedCtrlParam(val, 1);
    }
    else if ( ch1 == 'd')
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


void sendSettings()
{
  /*
    Serial.print("Settings:(kp,ki,kd):");
    Serial.print(mSettings.kp);
    Serial.print(",");
    Serial.print(mSettings.ki);
    Serial.print(",");
    Serial.println(mSettings.kd);
    Serial.print("Settings:(ato,usf,dfw):");
    Serial.print(mSettings.atObstacle);
    Serial.print(",");
    Serial.print(mSettings.unsafe);
    Serial.print(",");
    Serial.println(mSettings.dfw);
    Serial.print("V:");
    Serial.println(mSettings.velocity);
  */
}
void printMenu() {

  return;
  
//  Serial.println(F("\r\n========================================== Menu ==========================================\r\n"));
//  Serial.println(F("mu\t\t\t\tSend to show this menu\r\n"));
//  Serial.println(F("OF;\t\t\t\tSend to Stop. Send 'ON' again to continue\r\n"));
//
//  Serial.println(F("GP;\t\t\t\tGet PID values"));
//  Serial.println(F("GS;\t\t\t\tGet settings values"));
//  Serial.println(F("GI;\t\t\t\tGet info values\r\n"));
//
//  Serial.println(F("SP,Kp;\t\t\t\tUsed to set the Kp value"));
//  Serial.println(F("SI,Ki;\t\t\t\tUsed to set the Ki value"));
//  Serial.println(F("SD,Kd;\t\t\t\tUsed to set the Kd value"));
//  Serial.println(F("ST,targetAngle;\t\t\tUsed to set the target angle"));
//  Serial.println(F("SA,angle;\t\t\tUsed to set the maximum controlling angle"));
//  Serial.println(F("SU,value;\t\t\tUsed to set the maximum turning value"));
//  Serial.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));
//
//  Serial.println(F("IB;\t\t\t\tStart sending IMU values"));
//  Serial.println(F("IS;\t\t\t\tStop sending IMU values"));
//
//  Serial.println(F("RB;\t\t\t\tStart sending report values"));
//  Serial.println(F("RS;\t\t\t\tStop sending report values\r\n"));
//
//  Serial.println(F("CS;\t\t\t\tSend stop command"));
//  Serial.println(F("\r\n==========================================================================================\r\n"));
}


double pwm_to_ticks_r(double pwm, double dt);
double pwm_to_ticks_l(double pwm, double dt);

double vel_l_to_pwm( double vel);
double vel_r_to_pwm( double vel);


void sendTestInfo()
{

}


