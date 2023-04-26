///////////////////////////////////////////////////////////////
// CSFCROCO219.cpp class for inverted pendulum system
// ROCO219 practical
// Runs on Arduino Mega
// Implements observer based state feedback control with integral action
// operates with stepper motor connected via A4988 controller

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




vector<double> u(4);

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
  double   u = 0;
  double ymcn;
  
  // PUT YOUR OWN ComputeSFC FUNCTIONALITY IN HERE

  // Calculate time since last update
  double h = (double)(theTime - lastTime) / 1000.0;

  // compute control variable u 
  //u = -K*(xhat(1:4) - target) - Kp*(x(1:4) - target); % -K*(Xe)
  double uEstimate = -(xhat[0] * K[0] - target[0]) - (xhat[1] * K[1] - target[1]) -  (xhat[2] * K[2] - target[2]) -(xhat[3] * K[3] - target[3]); 
  double uReal = -(x[0] * Kp[0] - target[0]) - (x[1] * Kp[1] - target[1]) -  (x[2] * Kp[2] - target[2]) -(x[3] * Kp[3] - target[3])
  u =  uEstimate + uReal;

  
  
  // calculate observer correction term
    
  ymcn = (y - setPointAngle) - C[0] * xhat[0] - C[1] * xhat[1] - C[2] * xhat[2] - C[3] * xhat[3] ;
 

  // update the state estimates for xHat

  xhat[0] +=  h * (A[0][0] * xhat[0] + A[0][1] * xhat[1] + B[0] * u + L[0] * ymcn );
  xhat[1] +=  h * (A[1][0] * xhat[0] + A[1][1] * xhat[1] + B[1] * u + L[1] * ymcn );
  xhat[2] +=  h * (A[2][0] * xhat[0] + A[2][1] * xhat[1] + B[2] * u + L[2] * ymcn );
  xhat[3] +=  h * (A[3][0] * xhat[0] + A[3][1] * xhat[1] + B[3] * u + L[3] * ymcn );


  // use control velocity from input to update position of cart

  xhat[2] +=  h * (B[2] * u  );


  // compute integral action positional error state update



  // record variables

  lastTime = theTime;

  // return motor command
 return (u);
}




//ctrl = ctrb(A,B);
// Mc = rank(ctrl);

// %observability test
// %Mo = [C; CA;]
// observ = obsv(A,C);
// Mo = rank(observ);

// %calc the output 
// %y = X(c-DK)
// %there is no D matrix
// y = C * x(1:4);
// yHat =  C * xhat(1:4);

// %what we want the system to do
// target = [0; 0; pi; 0]; %we want everything to tend to 0 appart from the pendulum angle

// %steady state gain 
// Kp = place(A, B, [-4 -5 -6 -7]);


// %this is the 2 gains 
// u = -K*(x(1:4) - target) - Kp*(xhat(1:4) - target); % -K*(Xe)

// % using the real states (just for comparison)
// dx = A*x(1:4) + B*u;

// % using the estimate states
// % dx = Ax + Bu + L(y - yhat)
// % L(y - yhat) = L(output - output estimate)
// dxhatE = A*xhat(1:4) + B*u + L'*(y - yHat);

// xDot = dx; %store the new dx for both the real and simulated 
// xhatDot = dxhatE; %this is the estimated value 

