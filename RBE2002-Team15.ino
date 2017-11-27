#include <Wire.h>
#include <L3G.h>
#include <TimerOne.h>
#include "Drive.h"
#include "Stdfun.h"
#include "Turret.h"

Drive DriveTrain(6, 5);                   // Drive(int iRDrive, int iLDrive)
L3G gyro;                                 // L3G
SharpIR sFrontSonic(GP2YA41SK0F, A0);     // SharpIR(char* model, int Pin)
Turrent robotTurret(10);                  // Turret(int iFanPin)

double dSubData;
double dAngle = 0;
int iLastTime = 0;
int iCurTime = 0;

bool isCalibrated = false;

int rState = IdleCalibrate;

enum STATE{
  FindWall,
  IdleCalibrate,
  RightWallFollow,
  WallEdgeTurning,
  CornerTurning,
  ChickenHead,
  Triangulate,
  FlameApproach,
  Extinguish
};

void setup() {
  Serial.begin(9600);
  Wire.begin();

  DriveTrain.initDrive();
  DriveTrain.initTurnPID(1.5, 10, 1);
  DriveTrain.initWallFollowingPID(1, 1, 1);

  robotTurret.initTurret();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }

  gyro.enableDefault();
}

void loop() {
  iCurTime = millis();

  if((iCurTime - iLastTime) > 100){
    calcDegree();
  }

  switch(rState){
    case IdleCalibrate:
      // check if calibration has been done, if so then dont do it again
      if(dSubData == 0){
        // calibrate the gyro
        dSubData = calibrateGyro();
        
        // arm the fan
        /* FUNCTION TO ARM THE FAN */

        // set flag that the robot has been calibrated
        isCalibrated = true;
      }
      
    break;

    case FindWall:

    break;
  
    case RightWallFollow:

    break;
  
    case WallEdgeTurning:

    break;
  
    case CornerTurning:

    break;
  
    case ChickenHead:

    break;
  
    case Triangulate:

    break;
  
    case FlameApproach:

    break;
  
    case Extinguish:

    break;
  }

  // test new functions here
  if((iCurTime - iLastTime) > 10){
    //DriveTrain.FollowRightWall();
    //DriveTrain.TurnTo(90, dAngle);
  }

  // debug the angle /*REMOVE*/
  //Serial.println(dAngle);

  // set last time through the loop
  iLastTime = millis();
}

double calibrateGyro(){
  long total = 0;

  for(int i = 0; i < 370; i++){
    gyro.read();
    total += ((int)gyro.g.x);
    delay(100);
    Serial.println(total);
  }

  return total/350;
}

void calcDegree(){
  gyro.read();
  dAngle += ((gyro.g.x) - dSubData) * .0001;
}

void setDrive(){
  rState = FindWall;
}

