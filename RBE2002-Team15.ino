#include <Wire.h>
#include <L3G.h>
#include <SharpIR.h>
#include "Drive.h"
#include "Stdfun.h"
#include "Turret.h"
#include <Encoder.h>

Drive DriveTrain(6, 5);                   // Drive(int iRDrive, int iLDrive)
L3G gyro;                                 // L3G
SharpIR sFrontSonic(GP2Y0A21YK0F, A0);     // SharpIR(char* model, int Pin)GP2Y0A21YK0F
Turret RobotTurret(10);                   // Turret(int iFanPin)
Encoder rEncoder(2, 50);
Encoder lEncoder(3, 51);

// interrupt pin for the start and estop button
const int iInterruptPin = 18;

// pins for the right range finder
const int iREchoPin = 22;
const int iRTrigPin = 23;

// number of calibration cycles done by the gyro on startup
const int iGyroCalCycles = 1000;

// data to subrtract off of raw gyro values
double dSubData;
// current angle of the robot
double dAngle = 0;
// old angle
double dAngleOld = 0;

// current range of the front range finder
int iFrontRange = 0;
// current range of the right range finder
int iRightRange = 0;

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

// set angle to give to the PID loops
int iSetAngle = 0;
// set distance to give to the PID loops
int iSetDist = 5;
// set distance to give to PID loops from right wall
int iSetRDist = 3;
// last set distance on PID from wall
int iLastRDist = 1000;

int iThisCurDist = 0;

// flag if calibrated
bool isCalibrated = false;

// global x and y positions
double dXPosition = 0;
double dYPosition = 0;


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
  WallCorner3,
  WallCorner4,
};

// beginning state of the robot
int rState = IdleCalibrate;
int rLastState = FindWall;

// setup for the robot
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // initialization of the drive train stuff
  DriveTrain.initDrive();
  DriveTrain.initTurnPID(3, .001, 5);
  DriveTrain.initRWallPID(8, .001, 5);
  DriveTrain.initDistPID(6, .0004, 5);

  // initialization of the robot turret
  RobotTurret.initTurret();

  pinMode(iInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(iInterruptPin), setDrive, FALLING);

  // initialization of the gyro
  if (!gyro.init())
  {
    //Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }

  pinMode(iREchoPin, INPUT);
  pinMode(iRTrigPin, OUTPUT);
  
  gyro.enableDefault();
}

void loop() {
  // calculate the current time
  iCurTime = millis();

  // if the time is 100 milliseconds approx then recalcuate the sensor data
  if((iCurTime - iLastTime) > 100){
    calcDegree(); 
    // set last time through the loop
    iLastTime = iCurTime;
    calcRange();
  }

  // go through the state machine every 10 ms, not any faster
  if((iCurTime - iLastSwitchTime) > 10){
    switch(rState){
      case IdleCalibrate:
        // check if calibration has been done, if so then dont do it again
        if(dSubData == 0){
          // calibrate the gyro
          dSubData = calibrateGyro();

          // arm the fan
          /* FUNCTION TO ARM THE FAN */

          // set flag that the robot has been calibrated
          isCalibrated = true;

          rState = FindWall;
        }

        calcDistance();
        resetEncoderVal(&rEncoder, &lEncoder);
      break;

      case FindWall:
        // sensor fusion of gyro and front range finder and right range finder
        //DriveTrain.DriveToAngleDistanceFromRWall(iSetAngle, dAngle, iSetDist, iFrontRange, 0, 0);

        calcDistance();
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(iFrontRange - iSetDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = FindWall;
          iSetAngle = dAngle + 90;

          DriveTrain.resetPID();
        }
      break;

      case RightWallFollow:
        // sensor fusion of gyro and front range finder and right range finder
        DriveTrain.DriveToAngleDistanceFromRWall(iSetAngle, dAngle, iSetDist, iFrontRange, iSetRDist, iRightRange);

        calcDistance();
        resetEncoderVal(&rEncoder, &lEncoder);
        
        // count number of successes on this PID loop
        if(abs(iFrontRange - iSetDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // check to see if we came to wall edge
        if(iRightRange > 20){
          rState = WallCorner0;
          rLastState = RightWallFollow;
          iSetDist = 2;
        }else iLastRDist = iRightRange;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          rLastState = RightWallFollow;
          iSetAngle = dAngle + 90;

          DriveTrain.resetPID();
        }
      break;

      case CornerTurning:
        // turn to angle desired
        DriveTrain.TurnTo(iSetAngle, dAngle);
        
        if(abs(iSetAngle - dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses && rLastState == WallEdgeTurning){
          rState = WallEdgeTurning;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          iSetDist = 8;
          DriveTrain.resetPID();
        }else if(iSuccessCounter == iNumValidSuccesses && (rLastState == RightWallFollow || rLastState == FindWall)){
          rState = RightWallFollow;
          rLastState = CornerTurning;
          iSuccessCounter = 0;
          DriveTrain.resetPID();
          iLastRDist = 1000;
        }
      break;

      case ChickenHead:

      break;

      case Triangulate:

      break;

      case FlameApproach:

      break;

      case Extinguish:

      break;

      case WallCorner0:
        //Serial.println("Wall Corner");
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDistanceFromRWall(iSetAngle, dAngle, iThisCurDist, iSetDist, 0, 0);
         
        if(abs(iThisCurDist - iSetDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = WallCorner1;
          rLastState = WallCorner0;
          iSetAngle = dAngle - 90;

          calcDistance();
          resetEncoderVal(&rEncoder, &lEncoder);
          
          DriveTrain.resetPID();
        }
      break;

      case WallCorner1:
        // turn to angle desired
        DriveTrain.TurnTo(iSetAngle, dAngle);
        
        if(abs(iSetAngle - dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = WallCorner2;
          rLastState = WallCorner1;
          iSuccessCounter = 0;
          iSetDist = 5;
          DriveTrain.resetPID();
          
          resetEncoderVal(&rEncoder, &lEncoder);
        }
      break;

      case WallCorner2:
        iThisCurDist = returnDistance(&rEncoder);
        DriveTrain.DriveToAngleDistanceFromRWall(iSetAngle, dAngle, iThisCurDist, iSetDist, 0, 0);
         
        if(abs(iThisCurDist - iSetDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          if(iRightRange > 10){
            rState = WallCorner3;
            iSetAngle = dAngle - 90;
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
        // turn to angle desired
        DriveTrain.TurnTo(iSetAngle, dAngle);
        
        if(abs(iSetAngle - dAngle) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = FindWall;
          rLastState = WallCorner3;
          iSuccessCounter = 0;
          iSetDist = 5;
          DriveTrain.resetPID();
          
          resetEncoderVal(&rEncoder, &lEncoder);
        }
      break;

      default:
        exit(0);
      break;
    }

    iLastSwitchTime = iCurTime;
    //Serial.println(iRightRange);
  }

  Serial.print(" X: ");
  Serial.print(dXPosition);
  Serial.print(" Y: ");
  Serial.println(dYPosition);
//  Serial.print(" FrontRange: ");
//  Serial.print(iFrontRange);
//  Serial.print(" dAngle: ");
//  Serial.println(dAngle);
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
  dAngle = ((gyro.g.x) - dSubData) * .00955;
  dAngle = dAngle * .1;
  dAngle += dAngleOld;
  dAngleOld = dAngle;
}

void calcRange(){
  iFrontRange = (sFrontSonic.getDistance()) / 2.54;

  digitalWrite(iRTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(iRTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(iRTrigPin, LOW);

  int duration = pulseIn(iREchoPin, HIGH);
  iRightRange = (duration/74/2);
}

void setDrive(){
  //exit(0);
}

void calcDistance(){
  double dRTraveled  = returnDistance(&rEncoder);
  //double dLTraveled  = returnDistance(&lEncoder);
  double dBotTraveled = dRTraveled; //(dRTraveled + dLTraveled) / 2;
  double dAngleRad = dAngle * 0.0174533;
  
  dXPosition += dBotTraveled * cos(dAngleRad);
  dYPosition += dBotTraveled * sin(dAngleRad);
}
