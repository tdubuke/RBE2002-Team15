// Turret.cpp
// C++ file to hold all class information of the air turret

#include "Turret.h"

/**
 * Constructor
 */
Turret::Turret(int iFanPin, int iPanStepPin, int iPanDirPin, int ims1, int ims2, int ims3, int iTilt1, int iTilt2, int iTilt3, int iTilt4){
  iFanMotor = iFanPin;
  iPanStep = iPanStepPin;
  iPanDir = iPanDirPin;
  ms1 = ims1;
  ms2 = ims2;
  ms3 = ims3;
  tilt1 = iTilt1;
  tilt2 = iTilt2;
  tilt3 = iTilt3;
  tilt4 = iTilt4;
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

  iAngle = 90; //initialize iAngle to be inline with left of robot

  sTilt.initStepper(4096, tilt1, tilt2, tilt3, tilt4);
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
    dir = RIGHT;
  }
  else if(iAngle <= -45){
    dir = LEFT;
  }
  doStep(dir);
}

//On every call,
//Step the turret towards flameRead = 512
//where the turret aligns with front of robot
//return if flameRead is within error band of 512
boolean Turret::alignPan(int flameRead){
  //evaluate if the sensor value is within an acceptable band of error
  boolean lockedOn = (flameRead > (512 - ERROR_BAND) && flameRead < (512 + ERROR_BAND));

  //if locked on, return true and do nothing else
  if(lockedOn){
    return lockedOn; //true
  }
  //if we're not locked on, figure out which direction to turn
  else if(flameRead < 512){
    //meaning the flame is to the left of us
    doStep(LEFT);
  }
  else if(flameRead > 512){
    //the flame is to the right of us
    doStep(RIGHT);
  }
  return lockedOn; //false
  
}

void Turret::spinFan(){
  sESC.write(150);
}

double Turret::getAngle(){
  return iAngle;
}

//rotate the turret back to coordinate system zero
//return its current angle
double Turret::alignToZero(){
  if(iAngle == 0){
    return 0;
  }
  else if(iAngle > 0){
    //if we are on the left side of the sweep, go right
    doStep(RIGHT);
  }
  else if(iAngle < 0){
    //if we are on the right side of the sweep, go left
    doStep(LEFT);
  }
  return iAngle;
}

//do a whole step (1.8deg) in the given direction, goDirection
//update the global angle, iAngle
//return our resultant angle
double Turret::doStep(boolean goDirection){
  dir = goDirection;
  digitalWrite(ms1, LOW);
  digitalWrite(ms2, LOW);
  digitalWrite(ms3, LOW);
  
  if(goDirection == LEFT){
    digitalWrite(iPanDir,HIGH); //set stepper to left direction
    iAngle += STEP_ANGLE; //update the step angle
  }
  else if (goDirection == RIGHT){
    digitalWrite(iPanDir, LOW); //set stepper to right direction
    iAngle -= STEP_ANGLE; //update the step angle
  }
  //do one step
  digitalWrite(iPanStep,HIGH);
  digitalWrite(iPanStep,LOW);
  //return angle
  return iAngle;
}

//do a sixteenth step (0.2deg) in the given direction, goDirection
//update the global angle, iAngle
//return our resultant angle
double Turret::doSixteenthStep(boolean goDirection){
  dir = goDirection;
  digitalWrite(ms1, HIGH);
  digitalWrite(ms2, HIGH);
  digitalWrite(ms3, HIGH);
  
  if(goDirection == LEFT){
    digitalWrite(iPanDir,HIGH); //set stepper to left direction
    iAngle += SIXTEENTH_STEP_ANGLE; //update the step angle
  }
  else if (goDirection == RIGHT){
    digitalWrite(iPanDir, LOW); //set stepper to right direction
    iAngle -= SIXTEENTH_STEP_ANGLE; //update the step angle
  }
  //do one step
  digitalWrite(iPanStep,HIGH);
  digitalWrite(iPanStep,LOW);
  //return angle
  return iAngle;
}

//Using the tilt stepper motor
//do one step in
double Turret::doTilt(boolean goDirection){
  sTilt.step(1);
}



