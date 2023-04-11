///////////////////////////////////////////////////////////////
// SimpleEncoderTest program for inverted pendulum system
// runs on Arduino Mega
// operates with two phase incremental encoder
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

//////////////////////////////////////////////////
// setup the encoder and first include the encoder class
#include "Encoder.h"

// Define digital input pins for encoder that support interrupts 
// Green = A phase
// white = B phase
// red = power +ve
// black = power -ve
// digital connection pins used
int encoderPinA = 18;
int encoderPinB = 19;
// set the resolution

long encoderResolution = 4 * 600;
//long encoderResolution = 4 * 2000;

// Build the encoder object as global variable
Encoder myEnc(encoderPinA, encoderPinB);

/////////////////////////////////////////////////////////////
// Arduino setup function 
void setup() {
 // setup serial monitor so we can se serial the interface
  Serial.begin (9600);
}

/////////////////////////////////////////////////////////////
// flag to select output format
bool wantRawValue = true;
void loop() {

  // 100-ms delay so output doesn’t run too fast
  delay(100);

  // read encoder and print out values
  double encoderPosition = myEnc.read();

// Choose output format
if (wantRawValue) {
    // print out raw encoder values
    Serial.println(encoderPosition);
  }
  else {
    // convert raw value to radians
    double encoderAngle = encoderPosition * 2.0 * 3.142 / encoderResolution;
    // print out encoder angle in radians
    Serial.println(encoderAngle);
  }
}
