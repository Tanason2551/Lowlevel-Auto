#pragma once

#include "pid.h"

struct Pose2D {
  float x = 0.0f;       // World X (m)
  float y = 0.0f;       // World Y (m)
  float yawRad = 0.0f;  // World heading (rad, +CCW)
};

struct Goal2D {
  float x = 0.0f;  // Target world X (m)
  float y = 0.0f;  // Target world Y (m)
};

struct DistanceControlOutput {
  float vr = 0.0f;             // Commanded linear speed magnitude (m/s)
  float phiRad = 0.0f;         // Moving direction in robot frame (rad)
  float distanceError = 0.0f;  // S (m)
  float worldAngleRad = 0.0f;  // alpha (rad)
  bool atGoal = true;
};

/// Point-to-point distance controller (Phase 1: computes Vr and phi; omega=0).
class DistanceController {
 public:
  DistanceController(float kp, float ki, float kd, float integralLimit,
                     float maxLinearVel, float maxLinearAccel,
                     float posToleranceM);

  void reset();
  void setGains(float kp, float ki, float kd);
  DistanceControlOutput compute(const Pose2D& pose, const Goal2D& goal,
                                float dt);

 private:
  PIDController pid_;
  float maxLinearVel_;
  float maxLinearAccel_;
  float posToleranceM_;
  float lastVr_ = 0.0f;
};
