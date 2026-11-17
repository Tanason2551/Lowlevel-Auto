#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "joy_drive.h"
#include "kinematics.h"

namespace {

bool nearf(float a, float b, float eps = 1e-3f) {
  return fabsf(a - b) <= eps;
}

void testLinkTimeoutZeroesVelocity() {
  JoyDriveController joyCtrl(cfg::JOY_MAX_LINEAR_VEL, cfg::JOY_MAX_ANGULAR_VEL,
                             cfg::JOY_LINEAR_ACCEL, cfg::JOY_ANGULAR_ACCEL,
                             cfg::JOY_SIGN_VX, cfg::JOY_SIGN_VY,
                             cfg::JOY_SIGN_OMEGA);

  // Ramp up while link is alive (_Lx -> vx)
  const RobotVelocity active = joyCtrl.compute(JoyAxesInput{127, 0, 0, true}, 0.5f);
  assert(active.vx > 0.5f);

  // Link lost -> must drop to 0 immediately
  const RobotVelocity lost = joyCtrl.compute(JoyAxesInput{127, 0, 0, false}, 0.01f);
  assert(nearf(lost.vx, 0.0f));
  assert(nearf(lost.vy, 0.0f));
  assert(nearf(lost.omega, 0.0f));
  printf("[PASS] testLinkTimeoutZeroesVelocity\n");
}

void testAxisMappingSignsAndSlew() {
  JoyDriveController joyCtrl(cfg::JOY_MAX_LINEAR_VEL, cfg::JOY_MAX_ANGULAR_VEL,
                             cfg::JOY_LINEAR_ACCEL, cfg::JOY_ANGULAR_ACCEL,
                             cfg::JOY_SIGN_VX, cfg::JOY_SIGN_VY,
                             cfg::JOY_SIGN_OMEGA);

  // Slew-rate check on small dt (0.1s * 2.0 m/s^2 = 0.2 m/s step)
  const RobotVelocity step = joyCtrl.compute(JoyAxesInput{127, 0, 0, true}, 0.1f);
  assert(nearf(step.vx, 0.20f));

  // Full forward (_Lx = +127 -> +vx), full left (_Ly = +127 -> +vy via SIGN_VY=1),
  // full CCW (_Ry = -127 -> +omega via SIGN_OMEGA=-1)
  joyCtrl.reset();
  const RobotVelocity full =
      joyCtrl.compute(JoyAxesInput{127, 127, -127, true}, 1.0f);
  assert(nearf(full.vx, cfg::JOY_MAX_LINEAR_VEL));
  assert(nearf(full.vy, cfg::JOY_MAX_LINEAR_VEL));
  assert(nearf(full.omega, cfg::JOY_MAX_ANGULAR_VEL));
  printf("[PASS] testAxisMappingSignsAndSlew\n");
}

void testJoyWithKinematicsAndMotorInversion() {
  JoyDriveController joyCtrl(cfg::JOY_MAX_LINEAR_VEL, cfg::JOY_MAX_ANGULAR_VEL,
                             cfg::JOY_LINEAR_ACCEL, cfg::JOY_ANGULAR_ACCEL,
                             cfg::JOY_SIGN_VX, cfg::JOY_SIGN_VY,
                             cfg::JOY_SIGN_OMEGA);
  MecanumKinematics kin(cfg::WHEEL_RADIUS, cfg::HALF_LENGTH_X,
                        cfg::HALF_WIDTH_Y, cfg::MAX_WHEEL_RPM);

  // Push stick forward fully (_Lx = 127) -> ~0.55 m/s -> ~2017 motor RPM (left negative)
  const RobotVelocity vel = joyCtrl.compute(JoyAxesInput{127, 0, 0, true}, 1.0f);
  const WheelRPM wheel = kin.inverseKinematics(vel);

  const float flMotor = wheel.fl * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[0];
  const float frMotor = wheel.fr * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[1];
  const float rlMotor = wheel.rl * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[2];
  const float rrMotor = wheel.rr * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[3];

  assert(flMotor < -2000.0f && flMotor > -2030.0f);
  assert(rlMotor < -2000.0f && rlMotor > -2030.0f);
  assert(frMotor > 2000.0f && frMotor < 2030.0f);
  assert(rrMotor > 2000.0f && rrMotor < 2030.0f);

  // Pure CCW rotation (_Ry = -127 -> +omega):
  // Kinematics: FL < 0, FR > 0, RL < 0, RR > 0
  // After left-wheel inversion (-1): FL > 0, FR > 0, RL > 0, RR > 0
  joyCtrl.reset();
  const RobotVelocity rot = joyCtrl.compute(JoyAxesInput{0, 0, -127, true}, 1.0f);
  const WheelRPM rRot = kin.inverseKinematics(rot);
  assert(rRot.fl < 0.0f && rRot.rl < 0.0f && rRot.fr > 0.0f && rRot.rr > 0.0f);
  printf("[PASS] testJoyWithKinematicsAndMotorInversion\n");
}

}  // namespace

int main() {
  testLinkTimeoutZeroesVelocity();
  testAxisMappingSignsAndSlew();
  testJoyWithKinematicsAndMotorInversion();
  printf("All joy drive tests passed!\n");
  return 0;
}
