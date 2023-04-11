// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems 

// Plymouth University
// A324 Portland Square

// PL4 8AA
// Plymouth, Devon, UK
// howardlab.com

/*
   File:   CSFC3.h
   Author: ihoward

   Created on 20 April 2017, 21:41
*/

#ifndef CSFC3_H
#define CSFC3_H

class CSFC3 {
  public:

    // constuctor to setup all matrices
    CSFC3(double A[][3], double B[],  double C[], double K[], double L[], double setPointAngle);
    virtual ~CSFC3();

    // init the timers
    void InitSFC(unsigned long theTime);

    // compute SFC update
    double ComputeSFC(double InputAngle, unsigned long theTime);

  private:

// set max rank
  #define MAXRANK 3
  
    // SFC matrices
    double A[MAXRANK][MAXRANK];
    double B[MAXRANK];
    double C[MAXRANK];

    // rank of matrices
    int rank;

    // feedback gain
    double K[MAXRANK];

    // observer gain
    double L[MAXRANK];

    // target value
    double setPointAngle;

    // state estimate
    double xhat[MAXRANK];

    // last time
    unsigned long lastTime;
};

#endif /* CSFC3_H */

