#include "pid.h"

#include <math.h>

#include "math_utils.h"

PIDController::PIDController(float kp, float ki, float kd, float outMin,
                             float outMax, float integralLimit)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      outMin_(outMin),
      outMax_(outMax),
      integralLimit_(fabsf(integralLimit)) {}

void PIDController::reset() {
  integral_ = 0.0f;
  prevError_ = 0.0f;
  hasPrev_ = false;
}

void PIDController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

float PIDController::compute(float error, float dt) {
  if (dt <= 0.0f || !isfinite(error)) {
    return 0.0f;
  }

  integral_ = math_utils::clamp(integral_ + error * dt, -integralLimit_,
                                integralLimit_);

  float derivative = 0.0f;
  if (hasPrev_) {
    derivative = (error - prevError_) / dt;
  }
  prevError_ = error;
  hasPrev_ = true;

  const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return math_utils::clamp(output, outMin_, outMax_);
}
