// Turret.h
// H file to hold all class information of the air turret

#include <Servo.h>
#include <Arduino.h>

#ifndef TURRET_H
#define TURRET_H

#define LEFT true //CCW direction is LOW
#define RIGHT false //CW direction is HIGH

#define STEP_ANGLE 1.8

#define ERROR_BAND 28 //amount of tolerance on the flame read value

class Turret{
public:
  //initialize the turret with
  // - Fan Pin
  // - Pan Step Pin
  // - Pan Direction Pin
  // - Tilt Step Pin //wip
  // - Tilt Direction Pin //wip
  
  Turret(int iFanPin, int iPanStep, int iPanDir);
  void initTurret();
  void ArmFan();

  void spinFan();

  void doSweep();
  boolean alignPan(int flameRead);
  
  int getAngle();

  int alignToZero();
  
private:
  Servo sESC;

  int iFanMotor;
  int iPanStep;
  int iPanDir;

  boolean dir; // false is CCW, true is CW
  //will compare this to LEFT or RIGHT

  int iAngle;

  int doStep(boolean dir);
};

#endif
