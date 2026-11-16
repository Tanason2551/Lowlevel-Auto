#pragma once


class PIDController {
 public:
  PIDController(float kp, float ki, float kd, float outMin, float outMax,
                float integralLimit);

  void reset();
  void setGains(float kp, float ki, float kd);
  float compute(float error, float dt);

 private:
  float kp_;
  float ki_;
  float kd_;
  float outMin_;
  float outMax_;
  float integralLimit_;
  float integral_ = 0.0f;
  float prevError_ = 0.0f;
  bool hasPrev_ = false;
};
