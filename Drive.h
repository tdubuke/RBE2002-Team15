//TGD -- Drive.h

#include <Servo.h>
#include <Arduino.h>

#ifndef DRIVE_H
#define DRIVE_H

class Drive{
public:
  Drive(int iRDrive, int iLDrive);
  void initDrive();

  void resetPID();

  void initTurnPID(double iKP, double iKI, double iKD);
  void initRWallPID(double iKP, double iKI, double iKD);
  void initDistPID(double iKP, double iKI, double iKD);

  void DriveToAngleDistanceFromRWall(double iSetAngle, double iCurAngle, double iSetDist, double iCurDist, double iSetRightDist, double iCurRightDist);
  void DriveToAngleDeadReckoning(int dir, double iSetAngle, double iCurAngle, double iSetDist, double iCurDist, double iSetRightDist, double iCurRightDist);
  
  void TurnTo(double iSetAngle, double iCurAngle);
  int PIDTurn(double iSetAngle, double iCurAngle);

  void FollowRightWall(int iSetDist, int iCurDist, int iForSpeed);
  int PIDRightWall(int iSetDist, int iCurDist);

  void DriveTo(int iSetDist, int iCurDist);
  int PIDDistance(int iSetDist, int iCurDist);

  void StopMotors();
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
