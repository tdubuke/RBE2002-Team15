// Turret.h
// H file to hold all class information of the air turret

#include <Servo.h>
#include <Arduino.h>

#ifndef TURRET_H
#define TURRET_H

class Turret{
public:
  Turret(int iFanPin);
  void initTurret();
  
  void ArmFan();
private:
  Servo sESC;

  int iFanMotor;
};

#endif
