///////////////////////////////////////////////////////////////
// CStepper.cpp class for inverted pendulum system
// runs on Arduino Mega
// Implements velocity control of stepper
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

/*
   File:   CStepper.cpp
   Author: ihoward

   Created on 20 April 2017, 12:55
*/

#include "CStepper.h"
#include "TimerOne.h"

// pulse genrator ISR
// 50% duty cycle achieve by toggling output each time called

// define global this pointer for object access in callbacks
static CStepper * this_g;

// static stepper interrupt service callback routine
// this will  will generate pulses to drive the stepper motor
static void pulseISR()
{
  // run replay
  this_g->RunPulse();
}

///////////////////////////////////////////////////////////////
// construction
CStepper::CStepper(int stepPin, int dirPin, int enPin) {

  // Define pins used for stepper
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  this->enPin = enPin;

  // init signals variables
  pulseState = LOW;
  pulseCount = 0;
  cartLeftDirection = 1;

  // Setup digital output pins
  // setup stepPin pin for each stepper
  pinMode(stepPin, OUTPUT);
  // setup dirPin pin for each stepper
  pinMode(dirPin, OUTPUT);
  // setup enable pin for each stepper
  pinMode(enPin, OUTPUT);

  // record 'this' pointer to enable callback acccess to CStepper object
  this_g = this;
}

///////////////////////////////////////////////////////////////
// destruction
CStepper::~CStepper() {
}

///////////////////////////////////////////////////////////////
// setup the timer
void CStepper::SetupTimer()
{
  // Attach interrupt routine that will be called by timer
  // This Timer1 function must be called first
  // Argument in "microseconds" is the period of time the timer takes
  // run every 1 second
  Timer1.initialize(150000);
  // Attach interrupt routine that will be called by the timer
  Timer1.attachInterrupt(pulseISR);

  // set cartLeftDirection = 1
  SetStepperParams(150000, 1);
}

///////////////////////////////////////////////////////////////
// pulse generation function called from ISR
void CStepper::RunPulse() {

  // toggle pulse state each time function called
  if (pulseState == LOW) {
    pulseState = HIGH;
    // Inc or dec pulse count depending on direction
    if (cartLeftDirection == 1)
    {
      // count pulses
      pulseCount--;
    }
    else
    {
      // count pulses
      pulseCount++;
    }
  }
  else {
    pulseState = LOW;
  }
  // Write pulse state to digital output
  digitalWrite(stepPin, pulseState);
}

///////////////////////////////////////////////////////////////
// switch on steppers
void CStepper::EnableSteppers()
{
  // reset values
  pulseState = LOW;
  pulseCount = 0;

  // enable the outout
  digitalWrite(enPin, LOW);
}

///////////////////////////////////////////////////////////////
// switch off steppers
void CStepper::DisableSteppers()
{
  // disable the outputt
  digitalWrite(enPin, HIGH);
}

///////////////////////////////////////////////////////////////
// Set the stepper motor pulse period
void CStepper::SetStepperParams(long microseconds, int cartLeftDirection)
{
  // record direction
  this->cartLeftDirection = cartLeftDirection;

  // generate output
  if (cartLeftDirection == 1) {
    digitalWrite(dirPin, HIGH);
  }
  else
  {
    digitalWrite(dirPin, LOW);
  }

  // Set a new period
  Timer1.setPeriod(microseconds);
}


