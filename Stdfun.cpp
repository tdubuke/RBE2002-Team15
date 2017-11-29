// Stdfun.cpp
// File to be used to hold functions not part of a class
#include "Stdfun.h"
#include <Encoder.h>

void resetEncoderVal(Encoder eREncoder, Encoder eLEncoder){
  eREncoder.write(0);
  eLEncoder.write(0);
}
/*
void addToGlobalXPos(Encoder eREncoder, Encoder eREncoder, float ffXPosition){
  //change one to positive
  int iRXTick = eREncoder.read()
  int iLXTick = eLEncoder.read()

  float ffAvgTicks = (iRXTick + iLXTick) / 2;

  ffXPosition = ffXPosition + ffAvgTicks;


}
*/

float returnDistance(Encoder eEncoder){
  long ticks = eEncoder.read();

  float distance = ticks*2.75*PI;

  return distance;
}
