#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <SharpIR.h>
#include "Drive.h"
#include "Stdfun.h"
#include "Turret.h"
#include <Encoder.h>
#include <LiquidCrystal.h>

Drive DriveTrain(6, 5);                                     // Drive(int iRDrive, int iLDrive)
L3G gyro;                                                   // L3G
LSM303 accel;                                               // LSM303
SharpIR sFrontSonic(GP2YA41SK0F, A0);                      // SharpIR(char* model, int Pin)GP2Y0A21YK0F GP2Y0A21YK? GP2YA41SK0F
Turret RobotTurret(10, 25, 24, 31, 33, 35, 26, 28, 27, 29); // Turret(int iFanPin)
Encoder rEncoder(3, 51);                                    // Robot wheel encoder for odometry
Encoder lEncoder(2, 50);                                    // Robot wheel encoder for odometry
LiquidCrystal LCD(40,41,42,43,44,45);                       // LCD display initialization

const int iRightLine = 1;                   // saving the analog port of the right line sensor 
const int iLeftLine = 2;                    // saving the analog port of the left line sensor

// struct to hold the sensor data
struct SensorData{
  int iLightSensorX;
  int iLightSensorY;
  double iFrontRange;
  double iRightRange;
  int iLastRightRange;
  bool bIsCliff;
};

// struct to hold the set data
struct SetData{
  int iSetAngle;
  int iSetFrontDist;
  int iSetRightDist;
};

// struct to hold the global position data of things including robot and candle
struct GlobalPos{
  double dAngle;
  double dAngleOld;
  double dXPosition;
  double dYPosition;
  double dZPosition;
  bool valid;
};

// create the structs to hold all the positions
SensorData s_SensorData;
GlobalPos s_GlobalPos; //globalposition for current posn of bot
GlobalPos s_FirstSighting; //position of robot at first flame sighting
GlobalPos s_SecondSighting; //position of robot at second flame sighting
GlobalPos s_FlamePosition; //position of FLAME in our coordinate system
SetData s_SetData;

// for IR Sensor
/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
int IRsensorAddress = 0xB0;
int slaveAddress;

byte data_buf[16];
int i;
 
int Ix[4];
int Iy[4];
int s;
/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

// interrupt pin for the start and estop button
const int iInterruptPin = 18;

// pins for the right range finder
const int iREchoPin = 22;
const int iRTrigPin = 23;

// number of calibration cycles done by the gyro on startup
const int iGyroCalCycles = 1000;

// timer counter for the state machine
long iLastSwitchTime = 0;
// timer counter for the last sensor data
long iLastTime = 0;
// current time through the loop
long iCurTime = 0;

// success counter for PID loops
int iSuccessCounter = 0;
// number of successes needed to move to next stage
int iNumValidSuccesses = 4;

// int for keeping track of encoder distance
int iThisCurDist = 0;

// number to subtract off of gyro readings
double dSubData;

// flags for sightings
bool secondSightingFlag = false;
bool firstSightingFlag = false;

// flag if calibrated
bool isCalibrated = false;

// flag to see if we can stop when we return home
bool goHome = false;

// state machine enumeration
enum STATE{
  IdleCalibrate,
  FindWall,
  RightWallFollow,
  CornerTurning,
  AlignHead,
  ChickenHead,
  Triangulate,
  FlameApproach,
  FlameApproach2,
  Extinguish,
  WallCorner0,
  WallCorner1,
  WallCorner2,
  WallCorner3,
  BackOffCliff
};

// beginning state of the robot
int rState = IdleCalibrate;
int rLastState = IdleCalibrate;

// setup for the robot
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // set initial values for structs
  s_SetData.iSetAngle = 0;
  s_SetData.iSetFrontDist = 3;
  s_SetData.iSetRightDist = 3;

  s_SensorData.bIsCliff = false;

  s_GlobalPos.dAngle = 0;
  s_GlobalPos.dAngleOld = 0;
  s_GlobalPos.dXPosition = 0;
  s_GlobalPos.dYPosition = 0;

  s_FlamePosition.dXPosition = 0;
  s_FlamePosition.dYPosition = 0;
  s_FlamePosition.dZPosition = 0;

  s_FlamePosition.valid = false;

  // initialization of the drive train stuff
  DriveTrain.initDrive();
  DriveTrain.initTurnPID(1.1, 0.003, 2);
  DriveTrain.initRWallPID(2, .0003, 2.5);
  DriveTrain.initDistPID(7, 0.0004, 5);

  LCD.begin(16, 2);

  // initialization of the robot turret
  RobotTurret.initTurret();

  initLightSensor();
  Serial.println("Light Init success");

  pinMode(iREchoPin, INPUT);
  pinMode(iRTrigPin, OUTPUT);

  pinMode(iInterruptPin, INPUT_PULLUP);

  // initialization of the gyro
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }
  
  gyro.enableDefault();

  Accel_Init();
}

// if we see a flame see what we need to do with this information
void checkFlame(SensorData *s_SensorData, GlobalPos *s_GlobalPos, GlobalPos *s_FirstSighting, GlobalPos *s_SecondSighting){
  if(s_SensorData->iLightSensorY < 1023 && s_SensorData->iLightSensorX < 532 && s_SensorData->iLightSensorX > 492 && firstSightingFlag == false){
    
    s_FirstSighting->dAngle = s_GlobalPos->dAngle;
    s_FirstSighting->dAngleOld = s_GlobalPos->dAngleOld;
    s_FirstSighting->dXPosition = s_GlobalPos->dXPosition;
    s_FirstSighting->dYPosition = s_GlobalPos->dYPosition;
    s_FirstSighting->dZPosition = s_GlobalPos->dZPosition;
    firstSightingFlag = true;
    
  }else if(s_SensorData->iLightSensorY < 1023 && s_SensorData->iLightSensorX < 542 && s_SensorData->iLightSensorX > 482 && secondSightingFlag == true){
    // cleans angle and makes sure that new angle is perpendicular to first sightings angle
    if(abs(approxAngle(cleanAngle(s_GlobalPos->dAngle)) - approxAngle(cleanAngle(s_FirstSighting->dAngle))) == 90){
      s_SecondSighting->dAngle = s_GlobalPos->dAngle;
      s_SecondSighting->dAngleOld = s_GlobalPos->dAngleOld;
      s_SecondSighting->dXPosition = s_GlobalPos->dXPosition;
      s_SecondSighting->dYPosition = s_GlobalPos->dYPosition;
      s_SecondSighting->dZPosition = s_GlobalPos->dZPosition;
      rState = AlignHead;
      DriveTrain.StopMotors();
    }
  }
}

// helper function to make the angle nice to work with
int cleanAngle(double input){
  int returnVal = (int)input % 360;
  if(returnVal < 0) returnVal += 360;
  return returnVal; 
}

void loop() {
  // calculate the current time
  iCurTime = millis();

  // if the time is 100 milliseconds approx then recalcuate the sensor data
  if((iCurTime - iLastTime) > 100){
    // calculate the angle of the robot
    calcDegree(&s_GlobalPos); 
    // calculate the ranges of the range finders
    calcRange(&s_SensorData);
    // update the light sensor data
    updateLightValues(&s_SensorData);
    // update the lcd with the current values
    updateLCD(&s_SetData, &s_SensorData, &s_GlobalPos, &s_FlamePosition, rState);
    // update if we see a cliff
    calcCliff(&s_SensorData);

    // set last time through the loop
    iLastTime = iCurTime;
  }

  // go through the state machine every 10 ms, not any faster
  if((iCurTime - iLastSwitchTime) > 10){
    // if we are within 10 inches of home then stop
    if(goHome && abs(s_GlobalPos.dXPosition) < 5 && abs(s_GlobalPos.dYPosition) < 5){
      exit(0);
    }

    switch(rState){
      case IdleCalibrate:
        // check if calibration has been done, if so then dont do it again
        if(dSubData == 0){
          // calibrate the gyro
          dSubData = calibrateGyro();

          // set flag that the robot has been calibrated
          isCalibrated = true;
        }

        // if the button is pushed then go
        if(digitalRead(iInterruptPin) == 0){
          rState = FindWall;
          rLastState = IdleCalibrate;
        }
      break;

      case FindWall:
        // sensor fusion of gyro and front range finder and right range finder
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, s_SetData.iSetFrontDist, s_SensorData.iFrontRange, 0, 0);

        // calculate the distance traveled and reset the encoders for the next run
        calcDistance(&s_GlobalPos);
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(s_SensorData.iFrontRange - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if we see a cliff then do the appropriate action
        if(s_SensorData.bIsCliff){
          rState = BackOffCliff;
          rLastState = FindWall;

          s_SetData.iSetFrontDist = -2;
          DriveTrain.resetPID();
        }

        // if we see a wall within 7 inches then lock onto that
        if(s_SensorData.iRightRange < 7){
          rState = RightWallFollow;
          rLastState = FindWall;

          DriveTrain.resetPID();
        }

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = FindWall;
          
          s_SetData.iSetAngle += 90;

          DriveTrain.resetPID();
        }

        // check if we see a flame
        checkFlame(&s_SensorData, &s_GlobalPos, &s_FirstSighting, &s_SecondSighting);
      break;

      case RightWallFollow:
        // sensor fusion of gyro and front range finder and right range finder
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, 
                                                  s_SetData.iSetFrontDist, s_SensorData.iFrontRange, 
                                                  s_SetData.iSetRightDist, s_SensorData.iRightRange);

        // calculate the distance traveled and reset the encoders
        calcDistance(&s_GlobalPos);
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(s_SensorData.iFrontRange - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if we see a cliff then take the appropriate action
        if(s_SensorData.bIsCliff){
          rState = BackOffCliff;
          rLastState = RightWallFollow;

          s_SetData.iSetFrontDist = -2;
        }

        // check to see if we came to wall edge
        if(s_SensorData.iRightRange > 20){
          rState = WallCorner0;
          rLastState = RightWallFollow;
          s_SetData.iSetFrontDist = 4;
        }
        
        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = RightWallFollow;
          s_SetData.iSetAngle += 90;

          DriveTrain.resetPID();
        }

        // check to see if we see a flame
        checkFlame(&s_SensorData, &s_GlobalPos, &s_FirstSighting, &s_SecondSighting);
      break;

      case CornerTurning:
        // turn to angle desired
        DriveTrain.TurnTo(s_SetData.iSetAngle, s_GlobalPos.dAngle);

        // set the secondsighting availability flag
        if(firstSightingFlag == true && secondSightingFlag == false && rLastState != BackOffCliff){
          secondSightingFlag = true;
        }

        // reset the number of successes
        if(abs(s_SetData.iSetAngle - s_GlobalPos.dAngle) < 8) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if we came from right wall following or find wall or wall corner then go to right wall following
        if(iSuccessCounter == iNumValidSuccesses && (rLastState == RightWallFollow || rLastState == FindWall || rLastState == WallCorner2)){
          rState = RightWallFollow;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          resetEncoderVal(&rEncoder, &lEncoder);
          DriveTrain.resetPID();

        // if we came from align head then go to chicken head
        }else if(iSuccessCounter == iNumValidSuccesses && rLastState == AlignHead){
          rState = ChickenHead;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          resetEncoderVal(&rEncoder, &lEncoder);
          DriveTrain.resetPID();

        // if we came from back off cliff then go to find wall
        }else if(iSuccessCounter == iNumValidSuccesses && rLastState == BackOffCliff){
          rState = FindWall;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          resetEncoderVal(&rEncoder, &lEncoder);
          DriveTrain.resetPID();

        // if we came from flame approach then go to flame approach 2
        }else if(iSuccessCounter == iNumValidSuccesses && rLastState == FlameApproach){
          rState = FlameApproach2;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          resetEncoderVal(&rEncoder, &lEncoder);
          DriveTrain.resetPID();
        }
      break;

      case AlignHead:
        // make sure we still have the flame
        if(RobotTurret.alignTilt(s_SensorData.iLightSensorY) && RobotTurret.alignPan(s_SensorData.iLightSensorX) 
                                  && s_SensorData.iLightSensorX != 1023 && s_SensorData.iLightSensorY != 1023){
          rState = Triangulate;
          rLastState = AlignHead;
          
        // if we lose the flame then spray and pray
        }else if(s_SensorData.iLightSensorY == 1023){
          rState = Triangulate;
          rLastState = AlignHead;
        }
        
      break;

      case ChickenHead:
          
      break;

      case Triangulate:
        // calculate the flame move on
        flameCalcCorner(&s_FirstSighting, &s_SecondSighting, RobotTurret.getTiltAngle(), &s_FlamePosition);
        
        rState = Extinguish;
        rLastState = ChickenHead;
      break;

      case FlameApproach:
        // if we need to approach the flame then do it
        s_SetData.iSetAngle += RobotTurret.getAngle();
        rState = CornerTurning;
        rLastState = FlameApproach;
      break;

      case FlameApproach2:
        // align the head back to zero to face the flame
        if(RobotTurret.alignToZero() > -1.8 && RobotTurret.alignToZero() < 1.8){
          s_SetData.iSetFrontDist = s_SensorData.iFrontRange - 10;
          rState = FindWall;
          rLastState = FlameApproach2;
        }
      break;

      case Extinguish:
        // blow for 3 seconds, if we still see the flame, blow again
        if(s_SensorData.iLightSensorX < 1023 && s_SensorData.iLightSensorY < 1023){
          RobotTurret.spinFan();
          delay(3000);
          goHome = true;

        // else we continue on
        }else{
          RobotTurret.stopFan();
          rState = FindWall;
          rLastState = Extinguish;
        }
      break;

      case WallCorner0:
        // drive a set distance forward
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDeadReckoning(1, s_SetData.iSetAngle, s_GlobalPos.dAngle, iThisCurDist, s_SetData.iSetFrontDist, 0, 0);

        // if we see a cliff then take the appropriate actions
        if(s_SensorData.bIsCliff){
          rState = BackOffCliff;
          rLastState = WallCorner0;

          s_SetData.iSetFrontDist = -2;

          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);

          DriveTrain.resetPID();
        }

        // if the number of successes is sufficient, then continue onto next state
        if(iThisCurDist == s_SetData.iSetFrontDist){
          rState = WallCorner1;
          rLastState = WallCorner0;
          s_SetData.iSetAngle -= 90;

          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);
          
          DriveTrain.resetPID();
        }
      break;

      case WallCorner1:
        
        // turn to angle desired
        DriveTrain.TurnTo(s_SetData.iSetAngle, s_GlobalPos.dAngle);
        
        if(abs(s_SetData.iSetAngle - s_GlobalPos.dAngle) < 8) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = WallCorner2;
          rLastState = WallCorner1;
          iSuccessCounter = 0;
          s_SetData.iSetFrontDist = 18;
          DriveTrain.resetPID();
          resetEncoderVal(&rEncoder, &lEncoder);
        }
      break;

      case WallCorner2:
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDeadReckoning(1, s_SetData.iSetAngle, s_GlobalPos.dAngle, iThisCurDist, s_SetData.iSetFrontDist, 0, 0);

        if(s_SensorData.bIsCliff){
          rState = BackOffCliff;
          rLastState = WallCorner2;

          s_SetData.iSetFrontDist = -2;

          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);

          DriveTrain.resetPID();
        }

        if(s_SensorData.iFrontRange <= 5){
          rState = CornerTurning;
          rLastState = WallCorner2;

          s_SetData.iSetAngle += 90;

          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);

          DriveTrain.resetPID();
        }
         
        // if the number of successes is sufficient, then continue onto next state
        if(iThisCurDist == s_SetData.iSetFrontDist){
          if(s_SensorData.iRightRange > 5){
            rState = WallCorner3;
            rLastState = WallCorner2;
            s_SetData.iSetAngle -= 90;
          }else{
            rState = RightWallFollow;
            rLastState = WallCorner2;
          }
          
          rLastState = WallCorner2;
          
          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);

          DriveTrain.resetPID();
        }
      break;

      case WallCorner3:
        // turn to angle desired
        DriveTrain.TurnTo(s_SetData.iSetAngle, s_GlobalPos.dAngle);
        
        if(abs(s_SetData.iSetAngle - s_GlobalPos.dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = FindWall;
          rLastState = WallCorner3;
          iSuccessCounter = 0;
          s_SetData.iSetFrontDist = 5;
          DriveTrain.resetPID();
          
          resetEncoderVal(&rEncoder, &lEncoder);
        }
      break;

      case BackOffCliff:
        // drive the desired distance
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDeadReckoning(0, s_SetData.iSetAngle, s_GlobalPos.dAngle, iThisCurDist, s_SetData.iSetFrontDist, 0, 0);

        // if we are far enough back then turn and keep going
        if(iThisCurDist == s_SetData.iSetFrontDist){
          rState = CornerTurning;
          rLastState = BackOffCliff;
          
          s_SetData.iSetAngle += 90;
          s_SetData.iSetFrontDist = 4;
          
          calcDistance(&s_GlobalPos);
          resetEncoderVal(&rEncoder, &lEncoder);

          DriveTrain.resetPID();
        }
      break;

      default:
        exit(0);
      break;
    }

    // reset the last time through the loop so we maintain the 10 ms loops
    iLastSwitchTime = iCurTime;
  }
}

/**
 * Get the steady state error to subtract off gyroscope readings
 */
double calibrateGyro(){
  double total = 0;
  delay(1000); //Allow time for the gyro to settle

  Serial.println("Calibrating Gyro, Please Hold");

  // get data for gyro values to subtract off
  for(int i = 0; i < iGyroCalCycles; i++){
    gyro.read();
    total += gyro.g.x;
    delay(5);
  }

  Serial.println("Calibrated");
  return total/iGyroCalCycles;
}

/**
 * Initialize the accelerometer
 */
void Accel_Init(){
  accel.init();
  accel.enableDefault();
  Serial.print("Accel Device ID");
  Serial.println(accel.getDeviceType());
  switch (accel.getDeviceType())
  {
    case LSM303::device_D:
      accel.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      accel.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accel.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

/**
 * Calculate the angle of the robot at the current time
 * @param *s_GlobalPos Global Position to update
 */
void calcDegree(GlobalPos *s_GlobalPos){
  gyro.read();
  accel.read();

  double dAngle, dAngleOld;
  double x_Acc;

  double gyroX = gyro.g.x;
  
  double accelY = accel.a.y >> 4;
  double accelZ = accel.a.z >> 4;

  accelY /= 256;
  accelZ /= 256;
  
  s_GlobalPos->dAngle = ((gyroX) - dSubData) * .00875;
  s_GlobalPos->dAngle = s_GlobalPos->dAngle * .1;
  s_GlobalPos->dAngle += s_GlobalPos->dAngleOld;
  s_GlobalPos->dAngleOld = s_GlobalPos->dAngle;

  float magnitudeofAccel = (abs(accelY)+abs(accelZ));

  // use complimentary filter
  if(magnitudeofAccel < 6 && magnitudeofAccel > 1.2){
    x_Acc = atan2(accelY,accelZ)*180/ PI;
    s_GlobalPos->dAngle = s_GlobalPos->dAngle * 0.98 + x_Acc * 0.02;
  }

  // invert the angle so that rotating left is positive
  s_GlobalPos->dAngle *= -1;
}

/**
 * Calculate the right and forward ranges of the range finders
 * @param *s_SensorData Struct that holds all of the sensor data
 */
void calcRange(SensorData *s_SensorData){
  s_SensorData->iFrontRange = (sFrontSonic.getDistance());

  digitalWrite(iRTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(iRTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(iRTrigPin, LOW);

  int duration = pulseIn(iREchoPin, HIGH);
  s_SensorData->iRightRange = (duration/74.0/2.0);
}

/**
 * Update the IR cameras values
 * @param *s_SensorData Struct that holds all of the values of the sensor data
 */
void updateLightValues(SensorData *s_SensorData){
  //IR sensor read
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();

  // Request the 2 byte heading (MSB comes first)
  Wire.requestFrom(slaveAddress, 16);
  // Reset the array of 
  for (i=0;i<16;i++) { data_buf[i]=0; }
  i=0;
  while(Wire.available() && i < 16) {
      data_buf[i] = Wire.read();
      i++;
  }

  Ix[0] = data_buf[1];
  Iy[0] = data_buf[2];
  s     = data_buf[3];
  Ix[0] += (s & 0x30) <<4;
  Iy[0] += (s & 0xC0) <<2;

  s_SensorData->iLightSensorX = Ix[0];
  s_SensorData->iLightSensorY = Iy[0];
}

/**
 * Function to determine if we are located at a cliff
 * @param *s_SensorData Struct to hold the data of the sensors from the robot
 */
void calcCliff(SensorData *s_SensorData){
  if(analogRead(iRightLine) > 400 || analogRead(iLeftLine) > 400){
    s_SensorData->bIsCliff = true;
  }else{
    s_SensorData->bIsCliff = false;
  }
}

/**
 * Initialize the IR camera with writes to the camera
 */
void initLightSensor(){
  slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
  // IR sensor initialize
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);
  delay(100);
}

/**
 * Writes two bytes to the slave address
 * @param d1 The first byte to send
 * @param d2 The second byte to send
 */
void Write_2bytes(byte d1, byte d2)
{
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1); Wire.write(d2);
  Wire.endTransmission();
}

/**
 * Calculate the distance traveled based on the encoders
 * @param *s_GlobalPos The current global position of the robot
 */
void calcDistance(GlobalPos *s_GlobalPos){
  double dRTraveled  = returnDistance(&rEncoder);
  double dBotTraveled = dRTraveled;
  double dAngleRad = s_GlobalPos->dAngle * 0.0174533;
  
  s_GlobalPos->dXPosition += dBotTraveled * cos(dAngleRad);
  s_GlobalPos->dYPosition += dBotTraveled * sin(dAngleRad);
}


//Calculates the position of the flame by spotting flame, 
//Taking a corner,
//then spotting flame again
//consumes first position, s_FirstSighting
//second position, s_SecondSighting
//and the Y value from IR Camera at second sighting
//mutates position of the flame, write out to s_FlamePosition
void flameCalcCorner(GlobalPos *firstSight, GlobalPos *secondSight, double tiltAngle, GlobalPos *flamePosn){
  Serial.print("Tilt Angle = ");
  Serial.println(tiltAngle);
  Serial.print(firstSight->dXPosition);
  Serial.print(" ");
  Serial.print(firstSight->dYPosition);
  Serial.println(" ");
  Serial.print(secondSight->dXPosition);
  Serial.print(" ");
  Serial.println(secondSight->dYPosition);
  
  double firstAngle = (int)(firstSight->dAngle) % 360;
  double firstX = firstSight->dXPosition;
  double firstY = firstSight->dYPosition;
  
  double secondAngle = (int)(secondSight->dAngle) % 360;
  double secondX = secondSight->dXPosition;
  double secondY = secondSight->dYPosition;  double xFlame = 0;
  double yFlame = 0;
  double zFlame = 0;
  
  //according to the global coordinate system
  // 0deg   = NORTH  ^
  // 180deg = SOUTH  v
  // 270deg = EAST   <
  // 90deg  = WEST   >

  if(firstAngle < 0) firstAngle += 360;
  if(secondAngle < 0) secondAngle += 360; 

  //approximate angles to nearest 90 deg
  firstAngle = approxAngle(firstAngle);
  secondAngle = approxAngle(secondAngle);

  Serial.print(firstAngle);
  Serial.print(" ");
  Serial.println(secondAngle);

  //checking to see if something horrible has happened
  if((firstAngle == -1) || (secondAngle == -1)){
    //if we get unexpected garbage, send the robot looking again
    rState = FindWall;
    return;
  }
  // NORTH -> WEST
  else if(firstAngle == 0 && (secondAngle == 90 || secondAngle == 270)){
    xFlame = firstX;
    yFlame = secondY;
  }
  // WEST -> SOUTH
  else if(firstAngle == 90 && (secondAngle == 180 || secondAngle == 0)){
    xFlame = secondX;
    yFlame = firstY;
  }  
  // SOUTH -> EAST
  else if(firstAngle == 180 && (secondAngle == 270 || secondAngle == 90)){
    xFlame = firstX;
    yFlame = secondY;
  }  
  // EAST -> NORTH
  else if(firstAngle == 270 && (secondAngle == 0 || secondAngle == 180)){
    xFlame = secondX;
    yFlame = firstY;
  }else{
    rState = FindWall;
    return;
  }

  //compensate for posn of flame sensor being offset from the center of the IMU axis

  xFlame = adjust(xFlame);
  yFlame = adjust(yFlame);

  //calculate the height of the flame using trig
  zFlame = getFlameHeight(xFlame,  yFlame,  secondX,  secondY, tiltAngle);

  //set the values to the position object
  flamePosn->dXPosition = xFlame;
  flamePosn->dYPosition = yFlame;
  flamePosn->dZPosition = zFlame;

  Serial.print(flamePosn->dXPosition);
  Serial.print(" ");
  Serial.print(flamePosn->dYPosition);
  Serial.print(" ");
  Serial.println(flamePosn->dZPosition);

  flamePosn->valid = true;
}

//consumes input angle
//produces output angle approximated to the nearest 90degree
double approxAngle(double inputAngle){
  if( ((inputAngle >=315) && (inputAngle <= 360)) || ((inputAngle >= 0) && (inputAngle < 45))){
    return 0;
  }
  else if((inputAngle >= 45) && (inputAngle <= 134)){
    return 90;
  }
  else if((inputAngle >= 135) && (inputAngle <= 224)){
    return 180;
  }
  
  else if((inputAngle >= 225) && (inputAngle <= 315)){
    return 270;
  }
  else{ //something horrible has happened
    return -1;
  }
}

//consumes (X,Y) of flame,
// (X,Y) of second sighting of robot,
// and the angle read from the tilt stepper for the IR Camera to center on flame
double getFlameHeight(double xFlame, double yFlame, double xBot, double yBot, double tiltAngle){

  //we just want the magnitude of the difference;
  //the acutal direction of this does not matter, since we already know posn
  double deltaX = abs(abs(xFlame) - abs(xBot));
  double deltaY = abs(abs(yFlame) - abs(yBot));

  double d = 0; //initialize distance to flame
  double camHeight = 8.00; //height of the IR Camera from the floor

  d = sqrt(deltaX * deltaX + deltaY * deltaY);

  Serial.print("Distance To Flame: ");
  Serial.println(d);

  return camHeight + d * tan(tiltAngle * 0.0174533);    
}

//used to adjust xposn and yposn
//since IMU and flame sensor are 2.5" apart
double adjust(double input){
  double offset = 2.5;
  if(input <= 0){
    return input - offset;
  }
  else if(input > 0){
    return input + offset;
  } 
}

void updateLCD(SetData *s_SetData, SensorData *s_SensorData, GlobalPos *s_GlobalPos, GlobalPos *s_tFlamePosition, int stateString){
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("X:");
  LCD.setCursor(2, 0);
  LCD.print(s_GlobalPos->dXPosition);
  LCD.setCursor(8, 0);
  LCD.print("Y:");
  LCD.setCursor(10, 0);
  LCD.print(s_GlobalPos->dYPosition);

  LCD.setCursor(0, 1);

  if(rState == Extinguish){
    LCD.print("Extinguishing");
  }else if(s_tFlamePosition->valid){
    LCD.print((int)s_tFlamePosition->dXPosition);
    LCD.setCursor(5, 1);
    LCD.print((int)s_tFlamePosition->dYPosition);
    LCD.setCursor(10, 1);
    LCD.print((int)s_tFlamePosition->dZPosition);
  }else if(firstSightingFlag){
    LCD.print("FIRST SIGHTING");
  }else if(isCalibrated && rState == IdleCalibrate){
    LCD.print("Ready To Go Cap");
  }else{
    switch(stateString){
      case 0:
        LCD.print("Idle Calibrate"); 
        break;
      case 1:
        LCD.print("Find Wall"); 
        break;
      case 2:
        LCD.print("Right Wall"); 
        break;
      case 3:
        LCD.print("Corner Turning"); 
        break;
      case 4:
        LCD.print("Align Head"); 
        break;
      case 5:
        LCD.print("Chicken Head"); 
        break;
      case 6:
        LCD.print("Triangulate"); 
        break;
      case 7:
        LCD.print("Flame Approach"); 
        break;
      case 8:
        LCD.print("Flame Approach"); 
        break;
      case 9:
        LCD.print("Extinguish"); 
        break;
      case 10:
        LCD.print("Wall Corner");
        break;
      case 11:
        LCD.print("Wall Corner");
        break;
      case 12:
        LCD.print("Wall Corner");
        break;
      case 13:
        LCD.print("Wall Corner");
        break;
      case 14:
        LCD.print("Cliff");
        break;
      default:
        LCD.print("Searching Candle");
        break;
    }
  }
}
