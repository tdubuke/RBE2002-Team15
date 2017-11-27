//TGD -- Drive.h

#include <Servo.h>
#include <Arduino.h>

#ifndef DRIVE_H
#define DRIVE_H

class Drive{
public:
  Drive(int iRDrive, int iLDrive);
  void initDrive();

  void initTurnPID(int iKP, int iKI, int iKD);
  void TurnTo(int iSetAngle, int iCurAngle);
  int PIDTurn(int iSetAngle, int iCurAngle);

  void initRWallFollowingPID(int iKP, int iKI, int iKD);
  void FollowRightWall(int iSetDist, int iCurDist, int iForSpeed);
  int PIDRightWall(int iSetDist, int iCurDist);
  
private:
  int iRightDrivePin, iLeftDrivePin;
  
  Servo sRightDrive, sLeftDrive;

  double iTurnKP, iTurnKI, iTurnKD;
  double iTurnSumError;
  double iTurnLastError;

  double iRWallKP, iRWallKI, iRWallKD;
  double iRWallSumError;
  double iRWallLastError;
};

#endif
