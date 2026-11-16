#include "distance_control.h"

#include <math.h>

#include "math_utils.h"

DistanceController::DistanceController(float kp, float ki, float kd,
                                       float integralLimit, float maxLinearVel,
                                       float maxLinearAccel,
                                       float posToleranceM)
    : pid_(kp, ki, kd, 0.0f, fabsf(maxLinearVel), integralLimit),
      maxLinearVel_(fabsf(maxLinearVel)),
      maxLinearAccel_(fabsf(maxLinearAccel)),
      posToleranceM_(fabsf(posToleranceM)) {}

void DistanceController::reset() {
  pid_.reset();
  lastVr_ = 0.0f;
}

void DistanceController::setGains(float kp, float ki, float kd) {
  pid_.setGains(kp, ki, kd);
}

DistanceControlOutput DistanceController::compute(const Pose2D& pose,
                                                  const Goal2D& goal,
                                                  float dt) {
  DistanceControlOutput out;
  const float dx = goal.x - pose.x;
  const float dy = goal.y - pose.y;

  out.distanceError = hypotf(dx, dy);
  out.worldAngleRad = atan2f(dy, dx);
  out.phiRad = math_utils::wrapAngle(out.worldAngleRad - pose.yawRad);

  if (out.distanceError <= posToleranceM_) {
    out.atGoal = true;
    out.vr = 0.0f;
    pid_.reset();
    lastVr_ = 0.0f;
    return out;
  }

  out.atGoal = false;
  const float rawVr = math_utils::clamp(pid_.compute(out.distanceError, dt),
                                        0.0f, maxLinearVel_);
  lastVr_ = math_utils::slewRateLimit(lastVr_, rawVr, maxLinearAccel_, dt);
  out.vr = lastVr_;
  return out;
}
