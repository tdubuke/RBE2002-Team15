// Turret.cpp
// C++ file to hold all class information of the air turret

#include "Turret.h"

/**
 * Constructor
 */
Turret::Turret(int iFanPin, int iPanStepPin, int iPanDirPin){
  iFanMotor = iFanPin;
  iPanStep = iPanStepPin;
  iPanDir = iPanDirPin;
}

/**
 * Initialize the Turret
 */
void Turret::initTurret(){
  //attach servo object
  sESC.attach(iFanMotor, 1000, 2000);
  //set pan step pin to output
  pinMode(iPanStep, OUTPUT);
  //set pan direction pin to output
  pinMode(iPanDir, OUTPUT);

  ArmFan();
}

/**
 * Arming sequence for fan motor
 */
void Turret::ArmFan(){
  sESC.write(0);
  delay(4000);
  sESC.write(45);
  delay(3000);
}


//direction is kept track of in class
//when this is called, step once of proper direction
//also, update global angle
void Turret::doSweep(){
  if(iAngle >= 45){
    iAngle = 45;
    dir = LEFT;
  }
  else if(iAngle <= -45){
    iAngle = -45;
    dir = RIGHT;
  }
  
  if(dir == RIGHT){
    digitalWrite(8,HIGH);
    iAngle += STEP_ANGLE;
  }
  else if(dir == LEFT){
    digitalWrite(8,LOW);
    iAngle -= STEP_ANGLE;
  }
  
  digitalWrite(5,HIGH);
  digitalWrite(5,LOW);
}

//On every call,
//Step the turret towards iAngle = 0
//where the turret aligns with front of robot
//return if flameRead is within error band of 512
boolean Turret::alignPan(int flameRead){
  int errorBand = 4; //amount of tolerance on the read value
  
  if(flameRead < 512){
    //meaning the flame is to the left of us
    dir = LEFT;
    digitalWrite(8,LOW);
  }
  else if(flameRead > 512){
    //the flame is to the right of us
    dir = RIGHT;
    digitalWrite(8,HIGH);
  }
  //if there was a zero case, this would not have been called
  
  digitalWrite(5,HIGH);
  digitalWrite(5,LOW);

  return (flameRead > 512 - errorBand || flameRead < 512 + errorBand);
  
}


int Turret::getAngle(){
  return iAngle;
}

