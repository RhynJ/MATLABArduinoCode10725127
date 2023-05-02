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



#include <stdlib.h>
#include "CSFCROCO219.h"

////////////////////////////////////////////////////////////////
// construction passes system matrices, SFC and observer gains, as well as the setpoint location
CSFCROCO219::CSFCROCO219(double A[][4],  double B[], double C[], double K[], double L[], double setPointAngle) {

  // set rank for SFC
  rank = 4;
  rank2 = 2; // observer rank

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
  double u = 0.0;

  // Calculate time since last update
  // delta time in given in seconds
  double h = (double)(theTime - lastTime) / 1000.0;

  // compute control variable u 
  u = -K[0] * xhat[0] -K[1] * xhat[1] -K[2] * xhat[2] - K[3] * xhat[3];


  // calculate observer correction term
  yCorr = (y - setPointAngle) - C[0] * xhat[0] - C[1] * xhat[1];

  // update the state estimates for theta and thetaHat according to the equation
  // xhat = xhat + h * (A * xhat + B * u + L * (y - C * xhat) );
  // calculate state derivative xhatdot and sum up
  xhat[0] +=  h * (A[0][0] * xhat[0] + A[0][1] * xhat[1] + B[0] * u + L[0] * yCorr );
  xhat[1] +=  h * (A[1][0] * xhat[0] + A[1][1] * xhat[1] + B[1] * u + L[1] * yCorr );
  xhat[2] += h*(B[2]*u);
  

  xhatDot[0] += xhat[0] *h;
  xhatDot[1] += xhat[1] *h;



  // use control velocity from input to update position of cart
  //this is the x out this is the simulated 
  xdot[2] = u;
  xdot[3] = x[2];

  //this is the real
  x[2] +=  h * ( B[2] * u );
  x[3] +=  h * xdot[3];
  
  
  // record variables
  lastTime = theTime;

  // return motor command
  return (u);

}
