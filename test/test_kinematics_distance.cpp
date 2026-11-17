#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "distance_control.h"
#include "drivers/motor_can.h"
#include "kinematics.h"
#include "math_utils.h"

namespace {

bool nearf(float a, float b, float eps = 1e-3f) {
  return fabsf(a - b) <= eps;
}

void testMathUtils() {
  assert(nearf(math_utils::degToRad(180.0f), math_utils::PI_F));
  assert(nearf(math_utils::wrapAngle(math_utils::degToRad(185.0f)),
               math_utils::degToRad(-175.0f)));
  assert(nearf(math_utils::wrapAngle(math_utils::degToRad(-185.0f)),
               math_utils::degToRad(175.0f)));
  assert(nearf(math_utils::slewRateLimit(0.0f, 1.0f, 2.0f, 0.1f), 0.2f));
  printf("[PASS] testMathUtils\n");
}

void testMecanumKinematicsXConfig() {
  MecanumKinematics kin(cfg::WHEEL_RADIUS, cfg::HALF_LENGTH_X,
                        cfg::HALF_WIDTH_Y, cfg::MAX_WHEEL_RPM);

  // 1. Pure forward (+X): all 4 wheels equal and positive (before left invert)
  const RobotVelocity fwd = kin.movingSystem(0.5f, 0.0f, 0.0f);
  assert(nearf(fwd.vx, 0.5f));
  assert(nearf(fwd.vy, 0.0f));
  const WheelRPM rFwd = kin.inverseKinematics(fwd);
  const float expectedFwdRpm =
      (0.5f / cfg::WHEEL_RADIUS) * (60.0f / math_utils::TWO_PI_F);
  assert(nearf(rFwd.fl, expectedFwdRpm));
  assert(nearf(rFwd.fr, expectedFwdRpm));
  assert(nearf(rFwd.rl, expectedFwdRpm));
  assert(nearf(rFwd.rr, expectedFwdRpm));

  // 2. Pure strafe left (+Y): X-config signs (-, +, +, -)
  const RobotVelocity left =
      kin.movingSystem(0.5f, math_utils::PI_F / 2.0f, 0.0f);
  const WheelRPM rLeft = kin.inverseKinematics(left);
  assert(rLeft.fl < 0.0f && rLeft.fr > 0.0f && rLeft.rl > 0.0f &&
         rLeft.rr < 0.0f);
  assert(nearf(fabsf(rLeft.fl), fabsf(rLeft.fr)));

  // 3. Verify 0.55 m/s gives ~105.04 true wheel RPM (below 130 RPM cap)
  const RobotVelocity mid{0.55f, 0.0f, 0.0f};
  const WheelRPM rMid = kin.inverseKinematics(mid);
  assert(rMid.fl > 104.0f && rMid.fl < 106.0f);

  // 4. Proportional saturation preserves ratios when exceeding MAX_WHEEL_RPM
  const RobotVelocity tooFast{1.0f, 0.5f, 0.0f};
  const WheelRPM rSat = kin.inverseKinematics(tooFast);
  const float maxAbs = fmaxf(fmaxf(fabsf(rSat.fl), fabsf(rSat.fr)),
                             fmaxf(fabsf(rSat.rl), fabsf(rSat.rr)));
  assert(nearf(maxAbs, cfg::MAX_WHEEL_RPM));
  assert(nearf(rSat.fr / rSat.fl, (1.0f + 0.5f) / (1.0f - 0.5f)));
  printf("[PASS] testMecanumKinematicsXConfig\n");
}

void testDistanceControl() {
  DistanceController ctrl(cfg::DIST_KP, cfg::DIST_KI, cfg::DIST_KD,
                          cfg::DIST_INTEGRAL_LIMIT, cfg::MAX_LINEAR_VEL,
                          cfg::MAX_LINEAR_ACCEL, cfg::POS_TOLERANCE_M);

  // Robot faces +Y (90 deg), goal is at +X (1, 0): phi should be -90 deg
  const Pose2D pose{0.0f, 0.0f, math_utils::degToRad(90.0f)};
  const Goal2D goal{1.0f, 0.0f};
  const DistanceControlOutput out = ctrl.compute(pose, goal, 0.01f);
  assert(!out.atGoal);
  assert(nearf(out.distanceError, 1.0f));
  assert(nearf(out.phiRad, math_utils::degToRad(-90.0f)));
  assert(out.vr > 0.0f);

  // Inside POS_TOLERANCE_M -> atGoal & vr == 0
  const Pose2D nearGoal{0.99f, 0.0f, 0.0f};
  const DistanceControlOutput done = ctrl.compute(nearGoal, goal, 0.01f);
  assert(done.atGoal);
  assert(nearf(done.vr, 0.0f));
  printf("[PASS] testDistanceControl\n");
}

void testLeftWheelInversionAndCanProtocol() {
  MecanumKinematics kin(cfg::WHEEL_RADIUS, cfg::HALF_LENGTH_X,
                        cfg::HALF_WIDTH_Y, cfg::MAX_WHEEL_RPM);
  const WheelRPM raw = kin.inverseKinematics(RobotVelocity{0.55f, 0.0f, 0.0f});

  // Apply GEAR_RATIO * WHEEL_SIGN just like MotorCanDriver::setWheelRpms
  const float flCmd = raw.fl * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[0];
  const float frCmd = raw.fr * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[1];
  const float rlCmd = raw.rl * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[2];
  const float rrCmd = raw.rr * cfg::GEAR_RATIO * cfg::WHEEL_SIGN[3];

  assert(flCmd < -2000.0f && flCmd > -2030.0f);
  assert(rlCmd < -2000.0f && rlCmd > -2030.0f);
  assert(frCmd > 2000.0f && frCmd < 2030.0f);
  assert(rrCmd > 2000.0f && rrCmd < 2030.0f);
  assert(nearf(flCmd, -frCmd));

  // Verify MTDriver frame encoding & decoding
  const CAN_message_t rearTx =
      buildSetCurrentSpeed2(cfg::BOARD_ID_REAR, rlCmd, rrCmd);
  assert(rearTx.flags.extended == 1);
  assert(rearTx.len == 8);
  assert(rearTx.id == (0x65u | (0x02u << 8)));
  assert(nearf(unpackFloatBE(rearTx.buf), rlCmd));
  assert(nearf(unpackFloatBE(rearTx.buf + 4), rrCmd));

  // Verify status frame (0x08) parsing
  MotorBoardState boards[2];
  boards[0].node_id = cfg::BOARD_ID_REAR;
  boards[1].node_id = cfg::BOARD_ID_FRONT;

  CAN_message_t rx = makeExtendedFrame(cfg::BOARD_ID_FRONT, MT_CMD_MOTOR_STATUS, 8);
  rx.buf[0] = 0x04;
  rx.buf[1] = 0xD2;  // 1234 -> 12.34
  packFloatBE(rx.buf + 2, -75.5f);
  rx.buf[6] = 0xFF;
  rx.buf[7] = 0x9C;  // -100 -> -1.00 A

  assert(parseMotorStatus(rx, boards, 2, 12345));
  assert(boards[0].rx_count == 0);
  assert(boards[1].rx_count == 1);
  assert(boards[1].last_rx_ms == 12345);
  assert(nearf(boards[1].position, 12.34f));
  assert(nearf(boards[1].current_speed, -75.5f));
  assert(nearf(boards[1].current_amp, -1.0f));
  printf("[PASS] testLeftWheelInversionAndCanProtocol\n");
}

}  // namespace

int main() {
  testMathUtils();
  testMecanumKinematicsXConfig();
  testDistanceControl();
  testLeftWheelInversionAndCanProtocol();
  printf("All tests passed!\n");
  return 0;
}
