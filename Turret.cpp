// Turret.cpp
// C++ file to hold all class information of the air turret

#include "Turret.h"

/**
 * Constructor
 */
Turret::Turret(int iFanPin){
  iFanMotor = iFanPin;
}

/**
 * Initialize the Turret
 */
void Turret::initTurret(){
  sESC.attach(iFanMotor, 1000, 2000);
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

