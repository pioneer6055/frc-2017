/*
 * PID.cpp
 *
 *  Created on: Oct 15, 2016
 *      Author: chesterm
 */
#include "PID.h"

PID::PID(double* kP, double* kI, double* kD) {
  kP_ = kP;
  kI_ = kI;
  kD_ = kD;
  ResetError();
}
void PID::Initialize(double* kP, double* kI, double* kD) {
  kP_ = kP;
  kI_ = kI;
  kD_ = kD;
  ResetError();
}
void PID::ResetError() {
  errorSum_ = 0.0;
  lastError_ = 0.0;
}

double PID::Update(double goal, double currentValue) {
  double error = goal - currentValue;
  double p = *kP_ * error;
  errorSum_ += error;
  double i = *kI_ * errorSum_;
  double dError = error - lastError_;
  double d = *kD_ * dError;
  lastError_ = error;
  return p + i + d;
}



