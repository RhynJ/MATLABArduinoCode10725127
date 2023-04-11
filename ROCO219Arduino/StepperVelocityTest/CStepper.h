///////////////////////////////////////////////////////////////
// CStepper.h class for inverted pendulum system
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

/*
   File:   CStepper.h
   Author: ihoward
   Created on 20 April 2017, 12:55
*/

#ifndef CSTEPPER_H
#define CSTEPPER_H

class CStepper {
  public:
    CStepper(int stepPin, int dir, int enPin);
    virtual ~CStepper();
    void SetupTimer();
    void RunPulse();
    void EnableSteppers();
    void DisableSteppers();
    void SetStepperParams(long microseconds, int cartLeftDirection);

  private:
    bool pulseState;
    int stepPin;
    int dirPin;
    int enPin;
    
    // use volatile for variables used across threads
    volatile long pulseCount;
    volatile long cartLeftDirection;
    
};

#endif /* CSTEPPER_H */

