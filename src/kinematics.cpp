#include "kinematics.h"

#include <math.h>

#include "math_utils.h"

MecanumKinematics::MecanumKinematics(float wheelRadiusM, float halfLengthXM,
                                     float halfWidthYM, float maxWheelRpm)
    : wheelRadiusM_(wheelRadiusM),
      halfLengthXM_(halfLengthXM),
      halfWidthYM_(halfWidthYM),
      maxWheelRpm_(fabsf(maxWheelRpm)) {}

RobotVelocity MecanumKinematics::movingSystem(float vr, float phiRad,
                                              float omega) const {
  RobotVelocity vel;
  vel.vx = vr * cosf(phiRad);
  vel.vy = vr * sinf(phiRad);
  vel.omega = omega;
  return vel;
}

WheelRPM MecanumKinematics::inverseKinematics(const RobotVelocity& vel) const {
  if (wheelRadiusM_ <= 0.0f) {
    return WheelRPM{};
  }

  const float k = halfLengthXM_ + halfWidthYM_;
  const float radPerSecToRpm = 60.0f / math_utils::TWO_PI_F;

  WheelRPM rpm;
  rpm.fl = ((vel.vx - vel.vy - k * vel.omega) / wheelRadiusM_) * radPerSecToRpm;
  rpm.fr = ((vel.vx + vel.vy + k * vel.omega) / wheelRadiusM_) * radPerSecToRpm;
  rpm.rl = ((vel.vx + vel.vy - k * vel.omega) / wheelRadiusM_) * radPerSecToRpm;
  rpm.rr = ((vel.vx - vel.vy + k * vel.omega) / wheelRadiusM_) * radPerSecToRpm;


  if (maxWheelRpm_ > 0.0f) {
    float maxAbs = fabsf(rpm.fl);
    maxAbs = fmaxf(maxAbs, fabsf(rpm.fr));
    maxAbs = fmaxf(maxAbs, fabsf(rpm.rl));
    maxAbs = fmaxf(maxAbs, fabsf(rpm.rr));

    if (maxAbs > maxWheelRpm_) {
      const float scale = maxWheelRpm_ / maxAbs;
      rpm.fl *= scale;
      rpm.fr *= scale;
      rpm.rl *= scale;
      rpm.rr *= scale;
    }
  }

  return rpm;
}
