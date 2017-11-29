// Stdfun.h
// Used for Outside Functions that are not part of a class

#include <Encoder.h>
#include <Arduino.h>

void resetEncoders(Encoder *eREncoder, Encoder *eLEncoder);
//void addToGlobalXPos(Encoder eREncoder, Encoder eREncoder, float ffXPosition);
float returnDistance(Encoder *eEncoder);
