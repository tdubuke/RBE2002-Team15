//TGD -- Drive.h

#include <Servo.h>
#include <Arduino.h>

#ifndef DRIVE_H
#define DRIVE_H

class Drive{
public:
  Drive(int iRDrive, int iLDrive);
  void initDrive();

  void initTurnPID(double iKP, double iKI, double iKD);
  void initRWallFollowingPID(double iKP, double iKI, double iKD);
  void initDistPID(double iKP, double iKI, double iKD);

  void DriveToAngleDistance(int iSetAngle, int iCurAngle, int iSetDist, int iCurDist);
  
  void TurnTo(int iSetAngle, int iCurAngle);
  int PIDTurn(int iSetAngle, int iCurAngle);

  void FollowRightWall(int iSetDist, int iCurDist, int iForSpeed);
  int PIDRightWall(int iSetDist, int iCurDist);

  void DriveTo(int iSetDist, int iCurDist);
  int PIDDistance(int iSetDist, int iCurDist);
private:
  int iRightDrivePin, iLeftDrivePin;
  
  Servo sRightDrive, sLeftDrive;

  double iTurnKP, iTurnKI, iTurnKD;
  double iTurnSumError;
  double iTurnLastError;

  double iRWallKP, iRWallKI, iRWallKD;
  double iRWallSumError;
  double iRWallLastError;

  double iDistKP, iDistKI, iDistKD;
  double iDistSumError;
  double iDistLastError;
};

#endif
