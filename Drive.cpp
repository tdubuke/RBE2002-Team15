//TGD -- Drive.cpp

#include "Drive.h"

/**
 * Constructor for Drive object
 */
Drive::Drive(int iRDrive, int iLDrive){
  iRightDrivePin = iRDrive;
  iLeftDrivePin = iLDrive;
}

/**
 * Turn the motors off
 */
void Drive::StopMotors(){
  sRightDrive.write(90);
  sLeftDrive.write(90);
}

/**
 * Reset all the variables for the PID and turn off the motors
 */
void Drive::resetPID(){
  iTurnSumError = 0;
  iRWallSumError = 0;
  iDistSumError = 0;
  this->StopMotors();
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
void Drive::initRWallPID(double iKP, double iKI, double iKD){
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
void Drive::TurnTo(double iSetAngle, double iCurAngle){
  int iMotorOut = PIDTurn(iSetAngle, iCurAngle);

  if(iMotorOut > 30) iMotorOut = 30;
  else if(iMotorOut < -30) iMotorOut = -30;
  
  sRightDrive.write(90 + iMotorOut);
  sLeftDrive.write(90 + iMotorOut);
}

/**
 * Function to call when attempting to drive straight at a fixed angle a fixed distance from the right wall
 * @param iSetAngle The angle desired
 * @param iCurAngle The current angle of the robot
 * @param iSetDist The distance from the wall that is desired
 * @param iCurDist The distance from the wall that the robot is currently
 * @param iSetRightDist The distance from the right wall desired
 * @param iCurRightDist The distance from the right wall currently
 */
void Drive::DriveToAngleDistanceFromRWall(double iSetAngle, double iCurAngle, double iSetDist, double iCurDist, double iSetRightDist, double iCurRightDist){
  int iMotorOffset = PIDTurn(iSetAngle, iCurAngle);
  int iMotorSpeed = PIDDistance(iSetDist, iCurDist);
  int iMotorWallOffset = PIDRightWall(iSetRightDist, iCurRightDist);

  if(iMotorSpeed > 30) iMotorSpeed = 30;
  else if(iMotorSpeed < -30) iMotorSpeed = -30;
  
  if(iMotorOffset > 30) iMotorOffset = 30;
  else if(iMotorOffset < -30) iMotorOffset = -30;

  if(iMotorWallOffset > 30) iMotorWallOffset = 30;
  else if(iMotorWallOffset < -30) iMotorWallOffset = -30;
  
  sRightDrive.write(90 - iMotorSpeed + iMotorOffset + iMotorWallOffset);
  sLeftDrive.write(90 + iMotorSpeed + iMotorOffset + iMotorWallOffset);
}

/**
 * Function to call when attempting to drive straight at a fixed angle and fixed speed fixed distance from right wall
 * @param dir The direction to spin the motors
 * @param iSetAngle The angle desired
 * @param iCurAngle The current angle of the robot
 * @param iSetDist The distance from the wall that is desired
 * @param iCurDist The distance from the wall that the robot is currently
 * @param iSetRightDist The distance from the right wall desired
 * @param iCurRightDist The distance from the right wall currently
 */
void Drive::DriveToAngleDeadReckoning(int dir, double iSetAngle, double iCurAngle, double iSetDist, double iCurDist, double iSetRightDist, double iCurRightDist){
  int iMotorOffset = PIDTurn(iSetAngle, iCurAngle);
  int iMotorSpeed;
  int iMotorWallOffset = PIDRightWall(iSetRightDist, iCurRightDist);

  if(dir == 1) iMotorSpeed = -30;
  else if(dir == 0) iMotorSpeed = 30;

  if(iMotorOffset > 30) iMotorOffset = 30;
  else if(iMotorOffset < -30) iMotorOffset = -30;

  if(iMotorWallOffset > 30) iMotorWallOffset = 30;
  else if(iMotorWallOffset < -30) iMotorWallOffset = -30;
  
  sRightDrive.write(90 - iMotorSpeed + iMotorOffset + iMotorWallOffset);
  sLeftDrive.write(90 + iMotorSpeed + iMotorOffset + iMotorWallOffset);
}

/**
 * PID to determine motorOut adjustment for for following right wall
 * @param iSetDist The distance desired from the wall
 * @param iCurDist The distance actual from the wall
 * @return dMotorOut The value of motor speed adjustment
 */
int Drive::PIDRightWall(double iSetDist, double iCurDist){
  // calculate the error
  double iError = iSetDist - iCurDist;

  // calculate the derivative
  double dDer = (iError - iRWallLastError)/10;

  // total up the sum of the error but cap it at 100000
  if(iRWallSumError < 100000) iRWallSumError += (iError * 10);

  //save the last error
  iRWallLastError = iError;

  double motorOut = iRWallKP * iError + iRWallKI * iRWallSumError + iRWallKD + dDer;

  // return the current motorOut Data
  return (int)(motorOut);
}

/**
 * PID to determine motorOut adjustment for turning
 * @param iSetAngle The angle desired to be facing
 * @param iCurAngle The angle the robot is currently facing
 * @return The motor speed to turn
 */
int Drive::PIDTurn(double iSetAngle, double iCurAngle){
  // calculate the error
  double iError = iSetAngle - iCurAngle;

  // calculate the derivative
  double iDer = (iError - iTurnLastError)/10;

  // total up the sum of the error
  if(iTurnSumError < 2000 || iError < 0) iTurnSumError += (iError * 10);

  // save the last error
  iTurnLastError = iError;

  // calculate the motor out value
  double motorOut = iTurnKP * iError + iTurnKI * iTurnSumError + iTurnKD + iDer;

  // return the current motorOut Data
  return (int)(motorOut);
}

/**
 * PID to determine motorOut adjustment for driving to a wall
 * @param iSetDist The distance desired to be driving to
 * @param iCurDist The angle the robot is to drive to from the wall
 * @return The motor speed to drive
 */
int Drive::PIDDistance(double iSetDist, double iCurDist){
  // calculate the error
  double iError = iSetDist - iCurDist;

  // calculate the derivative
  double iDer = (iError - iDistLastError)/10;

  // total up the sum of the error
  if(iDistSumError < 100) iDistSumError += (iError * 10);

  // save the last error
  iDistLastError = iError;

  // calculate the motor out value
  double motorOut = iDistKP * iError + iDistKI * iDistSumError + iDistKD + iDer;

  // return the current motorOut Data
  return (int)(motorOut);
}
