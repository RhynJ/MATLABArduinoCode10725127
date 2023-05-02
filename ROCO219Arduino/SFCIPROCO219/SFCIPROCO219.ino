///////////////////////////////////////////////////////////////
// SFCIPROCO219 program for inverted pendulum system
// ROCO219 practical
// Implements SFC velocity control of inverted pendulum
// tendency for position to be maintained at starting location
// runs on Arduino Mega
// operates with stepper motor connected via A4988 controller
// Uses TimerOne PWM Pins Arduino Mega  11, 12, 13
// Uses Arduino Mega Interrupt pins: 2, 3, 18, 19, 20, 21
///////////////////////////////////////////////////////////////
// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems 
// Plymouth University
// A324 Portland Square
// PL4 8AA
// Plymouth,  Devon, UK
// howardlab.com
// 08/07/2017

//////////////////////////////////////////////////

// Include stepper class to achieve velocity control of stepper motor
#include "CStepper.h"

// SFC controller for 3x3 position augmented system matrix
#include "CSFCROCO219.h"

// include encoder
#include "Encoder.h"

//////////////////////////////////////////////////
// setup inverted pendulum systemt parameters

// select work pendulum
// set the resolution
double encoderResolution = 4 * 600;
//double encoderResolution = 4 * 2000;
//  set point - 0.6K version at work - depends on how stpper aligned with pendulum
double setPointAngle = 0.001;
// dead band to cut outout activity if very close to target - 0.6K version at work
long deadBandRange = 1;
// maxumum speed allowed -  need to clip to avoid stepper motor stalling
double maxSpeedValue = 0.6;
 
//////////////////////////////////////////////////
// setup the encoder

// control pins for encoder
// Green = A phase
// white = B phase
// red = power +ve
// black = power -ve
// digital connection pins used
int encoderPinA = 18;
int encoderPinB = 19;

// PP 2 degrees
double invertedStartRangeAngle = 0;
double invertedStartRangePulses = encoderResolution * invertedStartRangeAngle / 360;

// build the encoder object
Encoder myEnc(encoderPinA, encoderPinB);


////////////////////////////////////////////////////////////////
// correction direction

// positive for inverted pendulum
double correctionDirection = -1;

// pendulum equilibrium positon 
double equilibriumEncoderPostion = encoderResolution / 2;

////////////////////////////////////////////////////////////////
// setup stepper motor parameters

// for SMB302 pendulums
// Define the encoder pins control pins for A4988
int stepPin = 7;
int dirPin = 8;
int enPin = 6;

// control pins for A4988
//int stepPin = 5;
//int dirPin = 4;
//int enPin = 6;

// stepper accounting for 4x microstepping
double stepperPPR = 4 * 200;

// contruct stepper motor control obkject
CStepper myStepper(stepPin, dirPin, enPin);


// system matrix definitions
// the valuse here have been calculated in MATLAB
double A[4][4] = {{0, 1, 0, 0}, {13.1384,   -0.6664, 0 , 0},  {0, 0, 0, 0},  {0, 0, 1, 0}};
double B[4] = {1.3393, -0.8926, 1, 0};
double C[4] = {1, 0, 0, 0};




//state representation 
//angle
//angular velocity
//cart position
//error

double K[4] = {16.9239, 4.2605, -7.7297, -5.2478};
//these need to be + as it smothes out the response


// ENTER YOUR VALUES FOR THE OBSERVER GAINS HERE
// observer gain just for theta and thetadot
double L[2] = {29.3336, 14.7346};


////////////////////////////////////////////////////////////////
// build the state feedback control object
CSFCROCO219 mySFC( A, B, C, K, L, setPointAngle);

////////////////////////////////////////////////////////////////

// circumference of te pulley
double pulleyCircumference = 0.12;

// flag to indicate if pendulum control should continue
volatile bool pendulumActive;
volatile bool started;

// convert stepper pulses to linear distance of cart
double pulsesToDistance = 2 * stepperPPR / pulleyCircumference;

/////////////////////////////////////////////////////////////
// setup Hall endstop sensors
// for SMB302 pendulums
// control pins for Hall end stop sensors
int HallSensorLeft = 2;
int HallSensorRight = 3;

// Left Hall sensor callback
void HallLeftCB   (void)
{
  if (pendulumActive == false) {
    return;
  }

  // print message
  Serial.println("Left Hall Sensor activated");
  Serial.println("");

  // call function to deactivate control
  StopControl();
  myStepper.DisableSteppers();
}

// Right Hall sensor callback
void HallRightCB   (void)
{
  if (pendulumActive == false) {
    return;
  }

  // print message
  Serial.println("Right Hall Sensor activated");
  Serial.println("");

  // call function to deactivate control
  StopControl();
  myStepper.DisableSteppers();
}

// Associate Hall sensors to their callback functions
void SetupHall()
{
  // Setup callback input for Left Hall sensor
  pinMode(HallSensorLeft, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallSensorLeft), HallLeftCB, FALLING );

  // Setup callback input for Right Hall sensor
  pinMode(HallSensorRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallSensorRight), HallRightCB, FALLING );
}

/////////////////////////////////////////////////////////////
// control management

// Activate the control function
void ActivateControl()
{
  // init pulse count in stepper controller
  myStepper.Init();
    
  // Reset the current encoder position to zero
  myEnc.write(0);

  // init the SFC
  long int theTime = millis();
  mySFC.InitSFC(theTime);

  // havent started yet
  started = false;

  // turn on flag
  pendulumActive = true;
}

// Deactivate the control function
void StopControl()
{
  // Reset the current encoder position to zero
  myEnc.write(0);

  // havent started yet
  started = false;
  
  // turn off flag
  pendulumActive = false;
  
  // Set astepper update to a long period value
  // run every 1.5 second so can notice its still alive
  myStepper.SetStepperParams(1500000, 1);
}

/////////////////////////////////////////////////////////////
// print out help commands to serial output
void PrintHelp()
{
  // print help commands
  Serial.println("*** SFCInvPendTX ***");
  Serial.println("h: print help commands");
  Serial.println("o: switch off control");
  Serial.println("a: activate control");
  Serial.println("p: print parameters");
  Serial.println("");
}

////////////////////////////////////////////////////////////////
// print out all the inverted pendulum parameters
void PrintParameters()
{
  // print out A matrix
  Serial.println("*** A matrix ***");
  Serial.print("A[0][0] = ");
  Serial.println(A[0][0]);
  Serial.print("A[0][1] = ");
  Serial.println(A[0][1]);
  Serial.print("A[0][2] = ");
  Serial.println(A[0][2]);
  Serial.print("A[0][3] = ");
  Serial.println(A[0][3]);
  
  Serial.print("A[1][0] = ");
  Serial.println(A[1][0]);
  Serial.print("A[1][1] = ");
  Serial.println(A[1][1]);
  Serial.print("A[1][2] = ");
  Serial.println(A[1][2]);
  Serial.print("A[1][3] = ");
  Serial.println(A[1][3]);

  Serial.print("A[2][0] = ");
  Serial.println(A[2][0]);
  Serial.print("A[2][1] = ");
  Serial.println(A[2][1]);
  Serial.print("A[2][2] = ");
  Serial.println(A[2][2]);
  Serial.print("A[2][3] = ");
  Serial.println(A[2][3]);

  Serial.print("A[3][0] = ");
  Serial.println(A[3][0]);
  Serial.print("A[3][1] = ");
  Serial.println(A[3][1]);
  Serial.print("A[3][2] = ");
  Serial.println(A[3][2]);
  Serial.print("A[3][3] = ");
  Serial.println(A[3][3]);
  Serial.println("");

  // print out B matrix
  Serial.println("*** B matrix ***");
  Serial.print("B[0] = ");
  Serial.println(B[0]);
  Serial.print("B[1] = ");
  Serial.println(B[1]);
  Serial.print("B[2] = ");
  Serial.println(B[2]);
  Serial.print("B[3] = ");
  Serial.println(B[3]);
  Serial.println(" ");

  // print out C matrix
  Serial.println("*** C matrix ***");
  Serial.print("C[0] = ");
  Serial.println(C[0]);
  Serial.print("C[1] = ");
  Serial.println(C[1]);
  Serial.print("C[2] = ");
  Serial.println(C[2]);
  Serial.print("C[3] = ");
  Serial.println(C[3]);
  Serial.println("");

  // print out K matrix
  Serial.println("*** K matrix ***");
  Serial.print("K[0] = ");
  Serial.println(K[0]);
  Serial.print("K[1] = ");
  Serial.println(K[1]);
  Serial.print("K[2] = ");
  Serial.println(K[2]);
  Serial.print("K[3] = ");
  Serial.println(K[3]);
  Serial.println("");

  // print out L matrix
  Serial.println("*** L matrix ***");
  Serial.print("L[0] = ");
  Serial.println(L[0]);
  Serial.print("L[1] = ");
  Serial.println(L[1]);
  Serial.println("");

  // print out systems parameters
  Serial.println("*** systems parameters ***");
  Serial.print("setPointAngle [deg] = ");
  Serial.println(setPointAngle);
  Serial.print("pulleyCircumference [m] = ");
  Serial.println(pulleyCircumference);
  Serial.print("maxSpeedValue [m/s] = ");
  Serial.println(maxSpeedValue);
  Serial.print("encoderResolution (accounting for 4 edge operation) = ");
  Serial.println(encoderResolution);
  Serial.print("stepperPPR (accounting for 4x microstepping) = ");
  Serial.println(stepperPPR);
  Serial.print("invertedStartRangeAngle [deg] = ");
  Serial.println(invertedStartRangeAngle);
  Serial.println("");
}
////////////////////////////////////////////////////////////////
// poll loop function to decode menu input typed into the Arduino serial monitor
void PollControlMenuCommands()
{
  // check for incoming serial data:
  if (Serial.available() > 0) {

    // read incoming serial data:
    char inChar = Serial.read();

    // print out command
    Serial.print("Received command ");
    Serial.println(inChar);
    Serial.println("");

    // decode the command
    switch (inChar)
    {
      // print help commands
      case 'h':
        PrintHelp();
        break;
      // switch off steppers and control
      case 'o':
        // Disable pendulum
        StopControl();
        myStepper.DisableSteppers();
        Serial.println("Steppers disabled");
        break;
      // activate or reactivate steppers and control
      case 'a':
        // Enable pendulum
        myStepper.EnableSteppers();
        ActivateControl();
        Serial.println("Steppers enabled");
        break;
      case 'p':
        PrintParameters();
        Serial.println("Printed parameters");
        break;
    }
    Serial.println("");
  }
}

// if not started already look for start
void PollStart(double encoderPosition)
{
  // print out the encoder position until started
  // then dont as it takes up too much processing time
  Serial.println(encoderPosition);

  // get absolutte position of pendulum
  double encoderPositionA = abs(encoderPosition);

  // if  within range of starting position
  if (  (encoderPositionA <= (equilibriumEncoderPostion + invertedStartRangePulses))  &&  (encoderPositionA >= (equilibriumEncoderPostion - invertedStartRangePulses)))
  {

    // started flag now set
    started = true;
    Serial.println("Reached starting position");

    // measure time
    long int theTime = millis();

    // initialize the SFC controller
    mySFC.InitSFC(theTime);
  }
}

////////////////////////////////////////////////////////////////
// Utility function scales raw encoder output to radians
double ScaleInput(double encoderPosition)
{
  double encoderPositionE2;

  // if within range of equilibrium
  if (started)
  {
    // id started calculate angluar location of poll wrt unstable equilibrium condition
    if (encoderPosition >= 0 )
    {
      encoderPositionE2  = encoderPosition - equilibriumEncoderPostion;
    }
    else
    {
      encoderPositionE2  = encoderPosition + equilibriumEncoderPostion;
    }
  }
  // if not started then just compute wrt stable equilibrium
  else
  {
    encoderPositionE2 = encoderPosition;
  }

  // deadband
  if ( (int)(abs(encoderPositionE2)) < deadBandRange)
  {
    encoderPositionE2 = 0;
  }

  // scale angle from PPR to radians
  double inputAngle = encoderPositionE2 * 2.0 * PI / encoderResolution;
  return (inputAngle);
}

// first limit range of SFC output
// scale SFC output to microseconds for stepper motor speed control
long ScaleOutput(double SFCOutput)
{
  long microseconds;

  // limit maximum speed
  if (SFCOutput > maxSpeedValue) {
    SFCOutput = maxSpeedValue;
  }
  else if (SFCOutput < - maxSpeedValue)
  {
    SFCOutput = -maxSpeedValue;
  }
  // scale output velocity signal from m/s to pulses/s
  double velocity =  SFCOutput * pulsesToDistance;

  // calculate the pulse period
  microseconds = 1000000;
  if (abs(velocity) > 0) {
    microseconds = 1000000 / abs(velocity);
  }
  return (microseconds);
}

// Utility function to return direction of the stepper motor
int GetMotorDirection(double SFCOutput)
{
  int cartDirection;

  // determine the motor direction
  if (SFCOutput > 0) {
    cartDirection = 1;
  }
  else
  {
    cartDirection = 0;
  }
  return (cartDirection);
}

////////////////////////////////////////////////////////////////
// Arduino setup function 
void setup() {

  // start serial communications monitor
  Serial.begin (9600);

  // setup Hall endstop sensors
  SetupHall();

  // setup stepper timer routine
  myStepper.SetupTimer();

  // turn the steppers off
  myStepper.DisableSteppers();
  Serial.println("Steppers disabled");

  // initially switch the control off
  StopControl();

  // print help
  PrintHelp();
}

////////////////////////////////////////////////////////////////
// main poll loop
// THis runs as fast as possible and emasures time since last update
// uses Euler intergration in SFC to compute updates
void loop()
{
  // read the raw encoder value
  double encoderPosition = myEnc.read();

  // only run the SFC controller loop if active
  if (pendulumActive == true) {

    // Starts when desired equilibrium position reached
    if (started == false)
    {
      // if not started look for start condition
      PollStart(encoderPosition);
    }
    // Else if balancing started
    else
    {
      // otherwise run SFC poll loop
      // scale input to radians
      double inputAngle = ScaleInput(encoderPosition);

      // measure the time
      long int theTime = millis();

      // compute SFC output on the basis of pendulum angle
      double SFCOutput = mySFC.ComputeSFC(inputAngle, theTime);

      // set correction in appropriate direction
      SFCOutput = SFCOutput * correctionDirection;

      // Scale output to pulse duration in microseconds to achieve required velocity 
      // also limit SFC output
      long microseconds = ScaleOutput(SFCOutput);

      // get mnotor direction
      int cartDirection = GetMotorDirection(SFCOutput);

      // Set stepper motor period and direction
      myStepper.SetStepperParams(microseconds, cartDirection);
    }
  }

  // Regularly service command menu
  PollControlMenuCommands();
}
