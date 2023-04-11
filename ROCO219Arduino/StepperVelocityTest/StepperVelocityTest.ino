///////////////////////////////////////////////////////////////
// StepperVelocityTest program for inverted pendulum system
// Implements velocity control of stepper
// runs on Arduino Mega
// operates with stepper motor connected via A4988 controller
///////////////////////////////////////////////////////////////
// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems  
// Plymouth University
// A324 Portland Square 
// PL4 8AA
// Plymouth,   Devon,  UK
// howardlab.com
// 20/04/2017

// Include the stepper class
#include "CStepper.h"
#include "math.h"

////////////////////////////////////////////////////////////////
// Define the movement parameters

// set movement to 0.2Hz
double carriageFrequency = 0.2;

// set peak speed to 0.2m/s
double carriageAmplitide = 0.2;

// running time
double timeIdx;

////////////////////////////////////////////////////////////////
// stepper motor parameters

// Define the encoder pins control pins for A4988
int stepPin = 7;
int dirPin = 8;
int enPin = 6;

// Define stepper motor parameters
// stepper accouting for 4x microstepping
double stepperPPR = 4 * 200;
// size of drive pulley
double pulleyCircumference = 0.12;

// Build the velocity control stepper motor object
CStepper myStepper(stepPin, dirPin, enPin);


/////////////////////////////////////////////////////////////
// print out help commands to serial output
void PrintHelp()
{
  // print help commands
  Serial.println("*** StepperVelocityTest ***");
  Serial.println("h: print help commands");
  Serial.println("o: switch off stepper");
  Serial.println("a: activate stepper");
  Serial.println("p: print parameters");
  Serial.println("");
}

// print Parameters
void PrintParameters()
{

  // print out systems parameters
  Serial.println("*** systems parameters ***");
  Serial.print("carriageFrequency [Hz] = ");
  Serial.println(carriageFrequency);
  Serial.print("carriageAmplitide [m/s] = ");
  Serial.println(carriageAmplitide);
  Serial.print("pulleyCircumference [m]= ");
  Serial.println(pulleyCircumference);
  Serial.print("stepperPPR (accounting for 4x microstepping) = ");
  Serial.println(stepperPPR);
  Serial.println("");
}

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

    // decode the command
    switch (inChar)
    {
      // print help commands
      case 'h':
        PrintHelp();
        break;
      // switch off steppers and control
      case 'o':
        myStepper.DisableSteppers();
        Serial.println("Steppers deactivated");
        break;
      // activate or reactivate steppers and control
      case 'a':
        myStepper.EnableSteppers();
        Serial.println("Steppers activated");
        timeIdx = 0;
        break;
      case 'p':
        PrintParameters();
        Serial.println("Printed parameters");
        break;
    }
  }
}

// run the stepper speed controller loop
void PollStepperControl(double timeInc)
{
  double cartLeftDirection;

  // increment time
  timeIdx = timeIdx + timeInc;

  // calculate instantaneous velocity
  double omegaT =  2 * 2.0 * 3.142 * carriageFrequency * timeIdx;
  double shortTermVel =  carriageAmplitide * sin(omegaT);

  // determine the motor direction
  if (shortTermVel > 0) {
    cartLeftDirection = 1;
  }
  else
  {
    cartLeftDirection = 0;
  }

  // scale output velocity signal from m/s to pulses/s
  double velocity = 2 * stepperPPR * shortTermVel / pulleyCircumference;
  //Serial.println(velocity);

  // calculate the pulse period
  double microseconds = 1000000;
  if (abs(velocity) > 0) {
    microseconds = 1000000 / abs(velocity);
  }

  // Set stepper motor period and direction
  myStepper.SetStepperParams(microseconds, cartLeftDirection);
}

///////////////////////////////////////////////////////////////
// Arduino setup function 
void setup() {
  // setup serial I/O so can show output on monitor
  Serial.begin (9600);

  // setup stepper timer routine
  myStepper.SetupTimer();

  // turn the steppers off initially
  myStepper.DisableSteppers();
  Serial.println("Steppers disabled");

  // print help
  PrintHelp();
}

///////////////////////////////////////////////////////////////
// Arduino loop function 
void loop() {

  // Regularly poll the stepper controller
  PollStepperControl(0.01);

  // wait for a 10-ms delay so output target not computed too fast
  delay(10);

  // Regularly poll the command manager options menu
  PollControlMenuCommands();
}
