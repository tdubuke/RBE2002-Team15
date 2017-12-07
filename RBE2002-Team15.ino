#include <Wire.h>
#include <L3G.h>
#include <SharpIR.h>
#include "Drive.h"
#include "Stdfun.h"
#include "Turret.h"
#include <Encoder.h>
#include <LiquidCrystal.h>

Drive DriveTrain(6, 5);                   // Drive(int iRDrive, int iLDrive)
L3G gyro;                                 // L3G
SharpIR sFrontSonic(GP2Y0A21YK0F, A0);    // SharpIR(char* model, int Pin)GP2Y0A21YK0F
Turret RobotTurret(10, 25, 24);           // Turret(int iFanPin)
Encoder rEncoder(2, 50);                  // Robot wheel encoder for odometry
Encoder lEncoder(3, 51);                  // Robot wheel encoder for odometry
LiquidCrystal LCD(40,41,42,43,44,45);     // LCD display initialization

struct SensorData{
  int iLightSensorX;
  int iLightSensorY;
  int iFrontRange;
  int iRightRange;
  int iLastRightRange;
};

struct SetData{
  int iSetAngle;
  int iSetFrontDist;
  int iSetRightDist;
};

struct GlobalPos{
  double dAngle;
  double dAngleOld;
  double dXPosition;
  double dYPosition;
};

SensorData s_SensorData;
GlobalPos s_GlobalPos;
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
int iNumValidSuccesses = 20;

// int for keeping track of encoder distance
int iThisCurDist = 0;

// number to subtract off of gyro readings
double dSubData;

// flag if calibrated
bool isCalibrated = false;

// state machine enumeration
enum STATE{
  IdleCalibrate,
  FindWall,
  RightWallFollow,
  WallEdgeTurning,
  DriveToEncoder,
  CornerTurning,
  ChickenHead,
  Triangulate,
  FlameApproach,
  Extinguish,
  WallCorner0,
  WallCorner1,
  WallCorner2,
  WallCorner3
};

// beginning state of the robot
int rState = IdleCalibrate;
int rLastState = FindWall;

// setup for the robot
void setup() {
  Serial.begin(9600);
  Wire.begin();

  s_SetData.iSetAngle = 0;
  s_SetData.iSetFrontDist = 5;
  s_SetData.iSetRightDist = 3;

  s_GlobalPos.dAngle = 0;
  s_GlobalPos.dAngleOld = 0;
  s_GlobalPos.dXPosition = 0;
  s_GlobalPos.dYPosition = 0;

  // initialization of the drive train stuff
  DriveTrain.initDrive();
  DriveTrain.initTurnPID(3, .001, 5);
  DriveTrain.initRWallPID(10, .001, 5);
  DriveTrain.initDistPID(6, .0004, 5);

  LCD.begin(16, 2);

  // initialization of the robot turret
  RobotTurret.initTurret();

  initLightSensor();
  Serial.println("Light Init success");
  updateLCD("Light Init");

  pinMode(iInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(iInterruptPin), setDrive, FALLING);

  pinMode(iREchoPin, INPUT);
  pinMode(iRTrigPin, OUTPUT);

  // initialization of the gyro
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }
  
  gyro.enableDefault();
}

void loop() {
  // calculate the current time
  iCurTime = millis();

  // if the time is 100 milliseconds approx then recalcuate the sensor data
  if((iCurTime - iLastTime) > 100){
    // calculate the angle of the robot
    calcDegree(); 
    // set last time through the loop
    iLastTime = iCurTime;
    // calculate the ranges of the range finders
    calcRange();
    // update the light sensor data
    updateLightValues();

    s_SensorData.iLightSensorX = Ix[0];
    s_SensorData.iLightSensorY = Iy[0];
  }

  // go through the state machine every 10 ms, not any faster
  if((iCurTime - iLastSwitchTime) > 10){
    if(s_SensorData.iLightSensorX < 1023 || s_SensorData.iLightSensorY < 1023){
      rState = ChickenHead;
      DriveTrain.StopMotors();
    }

    switch(rState){
      case IdleCalibrate:
        // check if calibration has been done, if so then dont do it again
        if(dSubData == 0){
          // calibrate the gyro
          dSubData = calibrateGyro();

          // set flag that the robot has been calibrated
          isCalibrated = true;

          Serial.println("Ready to go Captain");
          updateLCD("Ready to go Cap");
        }
        
        if(digitalRead(iInterruptPin) == 0){
          rState = FindWall;
        }
      break;

      case FindWall:
        RobotTurret.doSweep();
        // sensor fusion of gyro and front range finder and right range finder
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, s_SetData.iSetFrontDist, s_SensorData.iFrontRange, 0, 0);

        calcDistance();
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(s_SensorData.iFrontRange - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = FindWall;
          s_SetData.iSetAngle = s_GlobalPos.dAngle + 90;

          DriveTrain.resetPID();
        }
      break;

      case RightWallFollow:
        RobotTurret.doSweep();
        // sensor fusion of gyro and front range finder and right range finder
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, 
                                                  s_SetData.iSetFrontDist, s_SensorData.iFrontRange, 
                                                  s_SetData.iSetRightDist, s_SensorData.iRightRange);

        calcDistance();
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(s_SensorData.iFrontRange - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // check to see if we came to wall edge
        if(s_SensorData.iRightRange > 10){
          rState = WallCorner0;
          rLastState = RightWallFollow;
          s_SetData.iSetFrontDist = 3;
        }else{
          s_SensorData.iLastRightRange = s_SensorData.iRightRange;
        }
        
        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = RightWallFollow;
          s_SetData.iSetAngle = s_GlobalPos.dAngle + 90;

          DriveTrain.resetPID();
        }
      break;

      case CornerTurning:
        // turn to angle desired
        DriveTrain.TurnTo(s_SetData.iSetAngle, s_GlobalPos.dAngle);
        
        if(abs(s_SetData.iSetAngle - s_GlobalPos.dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses && rLastState == WallEdgeTurning){
          rState = WallEdgeTurning;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          s_SetData.iSetFrontDist = 8;
          DriveTrain.resetPID();
        }else if(iSuccessCounter == iNumValidSuccesses && (rLastState == RightWallFollow || rLastState == FindWall)){
          rState = RightWallFollow;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          DriveTrain.resetPID();
          s_SensorData.iLastRightRange = 1000;
        }
      break;

      case ChickenHead:
        if(s_SensorData.iLightSensorX < 1023){
          if(RobotTurret.alignPan(s_SensorData.iLightSensorX)){
            RobotTurret.spinFan();
          }
        }else{
          exit(0);
        }
      break;

      case Triangulate:

      break;

      case FlameApproach:

      break;

      case Extinguish:

      break;

      case WallCorner0:
        RobotTurret.doSweep();
        //Serial.println("Wall Corner");
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, iThisCurDist, s_SetData.iSetFrontDist, 0, 0);
         
        if(abs(iThisCurDist - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = WallCorner1;
          rLastState = WallCorner0;
          s_SetData.iSetAngle = s_GlobalPos.dAngle - 90;

          calcDistance();
          resetEncoderVal(&rEncoder, &lEncoder);
          
          DriveTrain.resetPID();
        }
      break;

      case WallCorner1:
        RobotTurret.doSweep();
        // turn to angle desired
        DriveTrain.TurnTo(s_SetData.iSetAngle, s_GlobalPos.dAngle);
        
        if(abs(s_SetData.iSetAngle - s_GlobalPos.dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = WallCorner2;
          rLastState = WallCorner1;
          iSuccessCounter = 0;
          s_SetData.iSetFrontDist = 7;
          DriveTrain.resetPID();
          
          resetEncoderVal(&rEncoder, &lEncoder);
        }
      break;

      case WallCorner2:
        RobotTurret.doSweep();
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDistanceFromRWall(s_SetData.iSetAngle, s_GlobalPos.dAngle, iThisCurDist, s_SetData.iSetFrontDist, 0, 0);
         
        if(abs(iThisCurDist - s_SetData.iSetFrontDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          if(s_SensorData.iRightRange > 10){
            rState = WallCorner3;
            s_SetData.iSetAngle = s_GlobalPos.dAngle - 90;
          }else{
            rState = RightWallFollow;
          }
          rLastState = WallCorner2;
          iSuccessCounter = 0;
          DriveTrain.resetPID();
          
          calcDistance();
          resetEncoderVal(&rEncoder, &lEncoder);
        }
         
      break;

      case WallCorner3:
        RobotTurret.doSweep();
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

      default:
        exit(0);
      break;
    }

    iLastSwitchTime = iCurTime;
  }
  Serial.print(s_SensorData.iLightSensorX);
  Serial.print(" ");
  Serial.println(s_SensorData.iLightSensorY);
}

double calibrateGyro(){
  double total = 0;
  delay(1000); //Allow time for the gyro to settle

  Serial.println("Calibrating Gyro, Please Hold");

  for(int i = 0; i < iGyroCalCycles; i++){
    gyro.read();
    total += gyro.g.x;
    //Serial.println(total);
    delay(5);
  }

  Serial.println("Calibrated");
  return total/iGyroCalCycles;
}

void calcDegree(){
  gyro.read();
  s_GlobalPos.dAngle = ((gyro.g.x) - dSubData) * .00955;
  s_GlobalPos.dAngle = s_GlobalPos.dAngle * .1;
  s_GlobalPos.dAngle += s_GlobalPos.dAngleOld;
  s_GlobalPos.dAngleOld = s_GlobalPos.dAngle;
}

void calcRange(){
  s_SensorData.iFrontRange = (sFrontSonic.getDistance()) / 2.54;

  digitalWrite(iRTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(iRTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(iRTrigPin, LOW);

  int duration = pulseIn(iREchoPin, HIGH);
  s_SensorData.iRightRange = (duration/74/2);
}

void updateLightValues(){
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
}

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

void Write_2bytes(byte d1, byte d2)
{
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1); Wire.write(d2);
  Wire.endTransmission();
  Serial.println("light init success");
}

void setDrive(){
  //exit(0);
}

void calcDistance(){
  double dRTraveled  = returnDistance(&rEncoder);
  double dBotTraveled = dRTraveled;
  double dAngleRad = s_GlobalPos.dAngle * 0.0174533;
  
  s_GlobalPos.dXPosition += dBotTraveled * cos(dAngleRad);
  s_GlobalPos.dYPosition += dBotTraveled * sin(dAngleRad);
}

void updateLCD(String stateString){
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("X:");
  LCD.setCursor(2, 0);
  LCD.print(s_GlobalPos.dXPosition);
  LCD.setCursor(8, 0);
  LCD.print("Y:");
  LCD.setCursor(10, 0);
  LCD.print(s_GlobalPos.dYPosition);

  LCD.setCursor(0, 1);
  LCD.print(stateString); 
}

