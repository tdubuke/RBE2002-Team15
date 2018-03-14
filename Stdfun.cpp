// Stdfun.cpp
// File to be used to hold functions not part of a class
#include "Stdfun.h"

/**
 * Reset Encoders
 * @param *eREncoder The Pointer to the right encoder
 * @param *eLEncoder The Pointer to the left encoder
 */
void resetEncoderVal(Encoder *eREncoder, Encoder *eLEncoder){
  eREncoder->write(0);
  eLEncoder->write(0);
}

/**
 * Returns the distance that has been traveled by the encoder since the last reset in inches
 * @param *eEncoder The Pointer to the encoder to calculate
 */
double returnDistance(Encoder *eEncoder){
  double ticks = eEncoder->read();

  double distance = (ticks/3200.0) * 8.63937979;

  return distance;
}
