//TGD -- Drive.cpp

#include "Drive.h"

/**
 * Constructor for Drive object
 */
Drive::Drive(int iRDrive, int iLDrive){
  iRightDrivePin = iRDrive;
  iLeftDrivePin = iLDrive;
}

void Drive::resetPID(){
  iTurnSumError = 0;
  iRWallSumError = 0;
  iDistSumError = 0;
}

/**
 * Initialization for the turn PID values
 * @param iKP The P value for turning PID
 * @param iKI The I value for turning PID
 * @param iKD The D value for turning PID
 */
void Drive::initTurnPID(double iKP, double iKI, double iKD){
  iTurnKP = iKP;
  iTurnKI = iKI;
  iTurnKD = iKD;
}

/**
 * Initialization for the Right Wall Following PID values
 * @param iKP The P value for Right Wall Following PID
 * @param iKI The I value for Right Wall Following PID
 * @param iKD The D value for Right Wall Following PID
 */
void Drive::initRWallFollowingPID(double iKP, double iKI, double iKD){
  iRWallKP = iKP;
  iRWallKI = iKI;
  iRWallKD = iKD;
}

/**
 * Initialization for the Drive to wall PID values
 * @param iKP The P value for Drive to Wall PID
 * @param iKI The I value for Drive to Wall PID
 * @param iKD The D value for Drive to Wall PID
 */
void Drive::initDistPID(double iKP, double iKI, double iKD){
  iDistKP = iKP;
  iDistKI = iKI;
  iDistKD = iKD;
}

/**
 * Initialization of the Servos for driving
 */
void Drive::initDrive(){
  sRightDrive.attach(iRightDrivePin, 1000, 2000);
  sLeftDrive.attach(iLeftDrivePin, 1000, 2000);
}

/**
 * Function to call when wanting to PID turn to a certain angle
 * @param iSetAngle The angle desired
 * @param iCurAngle The current angle of the robot
 */
void Drive::TurnTo(int iSetAngle, int iCurAngle){
  int iMotorOut = PIDTurn(iSetAngle, iCurAngle);
  sRightDrive.write(90 + iMotorOut);
  sLeftDrive.write(90 + iMotorOut);
}

/**
 * Function to call when attempting to drive straight at a fixed angle
 * @param iSetAngle The angle desired
 * @param iCurAngle The current angle of the robot
 * @param iSetDist The distance from the wall that is desired
 * @param iCurDist The distance from the wall that the robot is currently
 */
void Drive::DriveToAngleDistance(int iSetAngle, int iCurAngle, int iSetDist, int iCurDist){
  int iMotorOffset = PIDTurn(iSetAngle, iCurAngle);
  int iMotorSpeed = PIDDistance(iSetDist, iCurDist);
  
  if(iMotorOffset > 20) iMotorOffset = 20;
  else if(iMotorOffset < -20) iMotorOffset = -20;
  
  sRightDrive.write(90 - iMotorSpeed + iMotorOffset);
  sLeftDrive.write(90 + iMotorSpeed + iMotorOffset);
}

/**
 * Function to call when wanting to PID drive to a wall
 * @param iSetDist The distance desired
 * @param iCurDist The current distance of the robot from the wall
 */
void Drive::DriveTo(int iSetDist, int iCurDist){
  int iMotorOut = PIDDistance(iSetDist, iCurDist);
  sRightDrive.write(90 - iMotorOut);
  sLeftDrive.write(90 + iMotorOut);
}

/**
 * Function to Call when wanting to PID follow a Right Wall
 * @param iSetDist The distance desired from the wall
 * @param iCurDist The distance currently from the wall
 * @param iForSpeed The forward moving speed when not adjusting
 */
void Drive::FollowRightWall(int iSetDist, int iCurDist, int iForSpeed){
  // calculate the motorOut data
  int iMotorOut = PIDRightWall(iSetDist, iCurDist);

  // adjust to match motor requirements
  if(iMotorOut > iForSpeed) iMotorOut = iForSpeed;
  else if(iMotorOut < iForSpeed*(-1)) iMotorOut = iForSpeed*(-1);

  // drive the motors
  sRightDrive.write(90 - iForSpeed + iMotorOut);
  sLeftDrive.write(90 + iForSpeed + iMotorOut);
}

/**
 * PID to determine motorOut adjustment for for following right wall
 * @param iSetDist The distance desired from the wall
 * @param iCurDist The distance actual from the wall
 * @return dMotorOut The value of motor speed adjustment
 */
int Drive::PIDRightWall(int iSetDist, int iCurDist){
  // calculate the error
  double iError = iSetDist - iCurDist;

  // calculate the derivative
  double dDer = (iError - iRWallLastError)/10;

  // total up the sum of the error
  iRWallSumError += (iError * 10);

  //save the last error
  iRWallLastError = iError;

  double motorOut = iRWallKP * iError + iRWallKI * iTurnSumError + iRWallKD + dDer;

  // return the current motorOut Data
  return (int)(motorOut);
}

/**
 * PID to determine motorOut adjustment for turning
 * @param iSetAngle The angle desired to be facing
 * @param iCurAngle The angle the robot is currently facing
 * @return The motor speed to turn
 */
int Drive::PIDTurn(int iSetAngle, int iCurAngle){
  // calculate the error
  double iError = iSetAngle - iCurAngle;

  // calculate the derivative
  double iDer = (iError - iTurnLastError)/10;

  // total up the sum of the error
  iTurnSumError += (iError * 10);

  // save the last error
  iTurnLastError = iError;

  // calculate the motor out value
  double motorOut = iTurnKP * iError + iTurnKI * iTurnSumError + iTurnKD + iDer;

  // adjust to the correct values
  if(motorOut > 90) motorOut = 90;
  else if(motorOut < -90) motorOut = -90;

  // return the current motorOut Data
  return (int)(motorOut);
}

/**
 * PID to determine motorOut adjustment for driving to a wall
 * @param iSetDist The distance desired to be driving to
 * @param iCurDist The angle the robot is to drive to from the wall
 * @return The motor speed to drive
 */
int Drive::PIDDistance(int iSetDist, int iCurDist){
  // calculate the error
  double iError = iSetDist - iCurDist;

  // calculate the derivative
  double iDer = (iError - iDistLastError)/10;

  // total up the sum of the error
  iDistSumError += (iError * 10);

  // save the last error
  iDistLastError = iError;

  // calculate the motor out value
  double motorOut = iDistKP * iError + iDistKI * iDistSumError + iDistKD + iDer;

  // adjust to the correct values
  if(motorOut > 90) motorOut = 90;
  else if(motorOut < -90) motorOut = -90;

  // return the current motorOut Data
  return (int)(motorOut);
}
