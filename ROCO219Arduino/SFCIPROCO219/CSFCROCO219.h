// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems 

// Plymouth University
// A324 Portland Square

// PL4 8AA
// Plymouth, Devon, UK
// howardlab.com

/*
   File:   CSFCROCO219.h
   Author: ihoward

   Created on 20 April 2017, 21:41
*/

#ifndef CSFCROCO219_H
#define CSFCROCO219_H

class CSFCROCO219 {
  public:

    // constuctor to setup all matrices
    CSFCROCO219(double A[][4], double A2[][2], double B[], double B2[],  double C[], double C2[], double K[], double L[], double setPointAngle);
    virtual ~CSFCROCO219();

    // init the timers
    void InitSFC(unsigned long theTime);

    // compute SFC update
    double ComputeSFC(double InputAngle, unsigned long theTime);

  private:

// set max rank
  #define MAXRANK 4
  #define MAXRANK2 2
  
    // SFC matrices
    double A[MAXRANK][MAXRANK];
    double B[MAXRANK];
    double C[MAXRANK];

    double A22[MAXRANK2][MAXRANK2];
    double B2[MAXRANK2];
    double C2[MAXRANK2];

    // rank of matrices
    int rank;
    int rank2;

    // feedback gain
    double K[MAXRANK];

    // observer gain
    double L[MAXRANK];

    // target value
    double setPointAngle;

    // state estimate
    double xhat[MAXRANK];
    double x[MAXRANK];
    double xhatDot[MAXRANK];
    double yCorr;
    double xdot[MAXRANK];

    // last time
    unsigned long lastTime;
};

#endif /* CSFCROCO219_H */

