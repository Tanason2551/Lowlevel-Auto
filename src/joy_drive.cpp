#include "joy_drive.h"

#include <math.h>

#include "math_utils.h"

namespace {
float normalizeAxis(int8_t raw) {
  return math_utils::clamp(static_cast<float>(raw) / 127.0f, -1.0f, 1.0f);
}
}  // namespace

JoyDriveController::JoyDriveController(float maxLinearVel, float maxAngularVel,
                                       float linearAccel, float angularAccel,
                                       int8_t signVx, int8_t signVy,
                                       int8_t signOmega)
    : maxLinearVel_(fabsf(maxLinearVel)),
      maxAngularVel_(fabsf(maxAngularVel)),
      linearAccel_(fabsf(linearAccel)),
      angularAccel_(fabsf(angularAccel)),
      signVx_(signVx >= 0 ? 1 : -1),
      signVy_(signVy >= 0 ? 1 : -1),
      signOmega_(signOmega >= 0 ? 1 : -1) {}

void JoyDriveController::reset() {
  lastVel_ = RobotVelocity{};
}

RobotVelocity JoyDriveController::compute(const JoyAxesInput& input, float dt) {
  // Fail-safe: stop immediately when Zigbee link drops or times out.
  if (!input.linkAlive) {
    reset();
    return RobotVelocity{};
  }

  const float targetVx =
      normalizeAxis(input.ly) * static_cast<float>(signVx_) * maxLinearVel_;
  const float targetVy =
      normalizeAxis(input.lx) * static_cast<float>(signVy_) * maxLinearVel_;
  const float targetOmega =
      normalizeAxis(input.rx) * static_cast<float>(signOmega_) * maxAngularVel_;

  lastVel_.vx =
      math_utils::slewRateLimit(lastVel_.vx, targetVx, linearAccel_, dt);
  lastVel_.vy =
      math_utils::slewRateLimit(lastVel_.vy, targetVy, linearAccel_, dt);
  lastVel_.omega =
      math_utils::slewRateLimit(lastVel_.omega, targetOmega, angularAccel_, dt);

  return lastVel_;
}
