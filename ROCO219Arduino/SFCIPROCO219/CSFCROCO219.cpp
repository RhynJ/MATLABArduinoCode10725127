///////////////////////////////////////////////////////////////
// CSFCROCO219.cpp class for inverted pendulum system
// ROCO219 practical
// Runs on Arduino Mega
// Implements observer based state feedback control with integral action
// operates with stepper motor connected via A4988 controller
///////////////////////////////////////////////////////////////
// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems
// Plymouth University
// A324 Portland Square
// PL4 8AA
// Plymouth, Devon, UK
// howardlab.com

///////////////////////////////////////////////////////////////
// state feedback controller with Luenberger observer
// state update performed using Euler integration
// See  p137
// Åström and Murray, Feedback Systems: An Introduction for Scientists and
// Engineers. Princeton University Press, 2008.
// http://www.cds.caltech.edu/~murray/amwiki/index.php/Main_Page

///////////////////////////////////////////////////////////////
// Observer based control
// Control algorithm - main loop - basic idea
//
// read setpoint
// r=in()
//
// read process variable
// y=in(encoder)
//
// compute control variable u
// u=C*x+Kr*r
//
// set  output
// out(u)
//
// update state estimate
// x=x+h*(A*x+B*u+L*(y-C*x))

#include <stdlib.h>
#include "CSFCROCO219.h"

////////////////////////////////////////////////////////////////
// construction passes system matrices, SFC and observer gains, as well as the setpoint location
CSFCROCO219::CSFCROCO219(double A[][4], double B[],  double C[], double K[], double L[], double setPointAngle) {

  // set rank for SFC
  rank = 4;

  // use a rank 2 observer!
  this->L[0] = L[0];
  this->L[1] = L[1];

  // for all rows
  for (int ridx = 0; ridx < rank; ridx++)
  {
    this->B[ridx] = B[ridx];
    this->C[ridx] = C[ridx];
    this->K[ridx] = K[ridx];

    // for all columns
    for (int cidx = 0; cidx < rank; cidx++)
    {
      this->A[ridx][cidx] = A[ridx][cidx];
    }
  }

  this->setPointAngle = setPointAngle;

  // init values
  lastTime = 0;
}

// destruction
CSFCROCO219::~CSFCROCO219() {

}

////////////////////////////////////////////////////////////////
// Setup the SFC
// sets the current time
void CSFCROCO219::InitSFC(unsigned long theTime)
{
  // record measure time
  this->lastTime = theTime;

  // init state
  // for all rows
  for (int ridx = 0; ridx < rank; ridx++)
  {
    xhat[ridx] = 0;
  }
}

////////////////////////////////////////////////////////////////
// compute the SFC
// given output angle of pendulum y and the current time
// returns the motor command u
double CSFCROCO219::ComputeSFC(double y, unsigned long theTime)
{
  // control value - stepper motor speed
  double   u = 0.0;
  
  // PUT YOUR OWN ComputeSFC FUNCTIONALITY IN HERE

  // Calculate time since last update

  // compute control variable u

  // calculate observer correction term

  // update the state estimates for xHat

  // use control velocity from input to update position of cart

  // compute integral action positional error state update

  // record variables

  // return motor command
 return (u);
}
