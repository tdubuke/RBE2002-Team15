#include <Wire.h>
#include "Drive.h"
#include <L3G.h>

Drive DriveTrain(6, 5);
L3G gyro;

double dSubData;
double dAngle = 0;
int iLastTime = 0;
int iCurTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  DriveTrain.initDrive();
  DriveTrain.initTurnPID(1.5, 10, 1);
  DriveTrain.initWallFollowingPID(1, 1, 1);

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }

  gyro.enableDefault();
  dSubData = calibrateGyro();
}

void loop() {
  iCurTime = millis();
  
  if((iCurTime - iLastTime) > 100){
    calcDegree();
  }

  if((iCurTime - iLastTime) > 10){
    DriveTrain.FollowRightWall();
    //DriveTrain.TurnTo(90, dAngle);
  }

  Serial.println(dAngle);

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
