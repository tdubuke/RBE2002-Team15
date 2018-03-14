// Stdfun.h
// Used for Outside Functions that are not part of a class

#include <Encoder.h>
#include <Arduino.h>

void resetEncoderVal(Encoder *eREncoder, Encoder *eLEncoder);
double returnDistance(Encoder *eEncoder);
