#pragma once

#include <stdint.h>

#include "kinematics.h"

struct JoyAxesInput {
  int8_t lx = 0;  // Left stick X (-127..127) -> strafe vy
  int8_t ly = 0;  // Left stick Y (-127..127) -> forward vx
  int8_t rx = 0;  // Right stick X (-127..127) -> rotate omega
  bool linkAlive = false;
};

/// Maps normalized PS4 joystick axes to robot-frame velocity (Vx, Vy, omega).
class JoyDriveController {
 public:
  JoyDriveController(float maxLinearVel, float maxAngularVel,
                     float linearAccel, float angularAccel, int8_t signVx,
                     int8_t signVy, int8_t signOmega);

  void reset();
  RobotVelocity compute(const JoyAxesInput& input, float dt);

 private:
  float maxLinearVel_;
  float maxAngularVel_;
  float linearAccel_;
  float angularAccel_;
  int8_t signVx_;
  int8_t signVy_;
  int8_t signOmega_;
  RobotVelocity lastVel_{};
};
