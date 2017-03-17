/*
 * PID.h
 *
 *  Created on: Oct 15, 2016
 *      Author: chesterm
 */

#ifndef PID_H_
#define PID_H_

class PID {
 public:
   PID(double* kP = 0, double* kI = 0, double* kD = 0);
  /**
   * Resets the error counts. Call when the PID loop is not active to prevent integral windup.
   */
  void Initialize(double* kP, double* kI, double* kD);
  void ResetError();

  double Update(double goal, double currentValue);

 private:
  double* kP_;
  double* kI_;
  double* kD_;

  // Cumulative error used in integral term
  double errorSum_;

  // Last error value used to find error difference for derivative term
  double lastError_;
};

#endif
