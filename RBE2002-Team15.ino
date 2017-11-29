#include <Wire.h>
#include <L3G.h>
#include <SharpIR.h>
#include "Drive.h"
#include "Stdfun.h"
#include "Turret.h"
#include <Encoder.h>

Drive DriveTrain(6, 5);                   // Drive(int iRDrive, int iLDrive)
L3G gyro;                                 // L3G
SharpIR sFrontSonic(GP2YA41SK0F, A0);     // SharpIR(char* model, int Pin)
Turret RobotTurret(10);                   // Turret(int iFanPin)
Encoder rEncoder(2, 3);
Encoder lEncoder(50, 52);

// interrupt pin for the start and estop button
const int iInterruptPin = 18;

// number of calibration cycles done by the gyro on startup
const int iGyroCalCycles = 50;

// data to subrtract off of raw gyro values
double dSubData;
// current angle of the robot
double dAngle = 0;
// current range of the front range finder
int iFrontRange = 0;

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
  CornerTurning,
  ChickenHead,
  Triangulate,
  FlameApproach,
  Extinguish
};

// beginning state of the robot
int rState = FindWall;

// setup for the robot
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // initialization of the drive train stuff
  DriveTrain.initDrive();
  DriveTrain.initTurnPID(2, .0001, 3);
  //DriveTrain.initRWallFollowingPID(1, 1, 1);
  DriveTrain.initDistPID(5, .001, 5);

  // initialization of the robot turret
  RobotTurret.initTurret();

  pinMode(iInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(iInterruptPin), setDrive, FALLING);

  // initialization of the gyro
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type! Turn on the Robot Dumbass");
    while (1);
  }

  // calibrate the subData /* TO BE MOVED TO CALIBRATION STATE OF STATE MACHINE */
  dSubData = calibrateGyro();
  gyro.enableDefault();
}

void loop() {
  // calculate the current time
  iCurTime = millis();

  // if the time is 100 milliseconds approx then recalcuate the sensor data
  if((iCurTime - iLastTime) > 100){
    calcDegree();
    calcRange();

    // set last time through the loop
    iLastTime = iCurTime;
  }

  // go through the state machine every 10 ms, not any faster
  if((iCurTime - iLastSwitchTime) > 10){
    switch(rState){
      case IdleCalibrate:
        // check if calibration has been done, if so then dont do it again
        if(dSubData == 0){
          // calibrate the gyro
          //dSubData = calibrateGyro();

          // arm the fan
          /* FUNCTION TO ARM THE FAN */

          // set flag that the robot has been calibrated
          isCalibrated = true;
        }
      break;

      case FindWall:
        /*PLACE HOLDER, TO BE USED IF NECESSARY*/
        rState = RightWallFollow;
      break;

      case RightWallFollow:
        // sensor fusion of gyro and front range finder
        //Serial.println("RightWallFollowing");
        DriveTrain.DriveToAngleDistance(iSetAngle, dAngle, iSetDist, iFrontRange);
        // count number of successes on this PID loop
        if(abs(iFrontRange - iSetDist) < 2) iSuccessCounter++;
        else iSuccessCounter = 0;

        // if the number of successes is sufficient, then continue onto next state
        if(iSuccessCounter == iNumValidSuccesses){
          iSuccessCounter = 0;
          rState = CornerTurning;
          iSetAngle += 90;

          DriveTrain.resetPID();
        }
      break;

      case WallEdgeTurning:

      break;

      case CornerTurning:
        // turn to angle desired
        //Serial.println("CornerTurning");
        DriveTrain.TurnTo(iSetAngle, dAngle);
        if(abs(iSetAngle - dAngle) < 5) iSuccessCounter++;
        else iSuccessCounter = 0;

        if(iSuccessCounter == iNumValidSuccesses){
          rState = RightWallFollow;
          iSuccessCounter = 0;
          DriveTrain.resetPID();
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
    }

    iLastSwitchTime = iCurTime;
  }

  // test new functions here
  //if((iCurTime - iLastTime) > 10){
  //  DriveTrain.TurnTo(setAngle, dAngle);
  //  DriveTrain.DriveTo(setDist, iFrontRange);
  //}

  // debug the angle /*REMOVE*/
  Serial.print(dAngle);
  Serial.print(" ");
  Serial.print(iSetAngle);
  Serial.print(" ");
  Serial.println(returnDistance(&rEncoder));
}

double calibrateGyro(){
  long total = 0;

  Serial.println("Calibrating Gyro, Please Hold");

  for(int i = 0; i < iGyroCalCycles; i++){
    gyro.read();
    total += ((int)gyro.g.x);
    delay(100);
    Serial.println(total);
  }

  Serial.println("Calibrated");
  return total/iGyroCalCycles;
}

void calcDegree(){
  gyro.read();
  dAngle += ((gyro.g.x) - dSubData) * .0001 * 9;
}

void calcRange(){
  iFrontRange = sFrontSonic.getDistance();
}

void setDrive(){
//  if(rState == IdleCalibrate){
//    rState = FindWall;
//    Serial.println("Going to Right Wall Following");
//  }else{
    Serial.println("E-stop");
    exit(0);
 // }
}

void calcDistance(){
  double dRTraveled  = returnDistance(&rEncoder);
  double dLTraveled  = returnDistance(&lEncoder);
  double degrees = dAngle;
  double dBotTraveled = (dRTraveled + dLTraveled) / 2;

  dXPosition += dBotTraveled * cos(degrees);
  dYPosition += dBotTraveled * sin(degrees);
}
