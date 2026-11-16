#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <stdint.h>

#include "config.h"
#include "drivers/motor_can.h"
#include "kinematics.h"

/// Manages FlexCAN_T4 communication with 2 MTDriver Set-A boards (4 wheels).
/// Applies left-wheel sign inversion (FL, RL * -1) prior to CAN transmission.
class MotorCanDriver {
 public:
  MotorCanDriver();

  void begin();
  void update();

  /// Apply wheel polarity (WHEEL_SIGN) and stage RPMs for Rear(0x65) & Front(0x66).
  void setWheelRpms(const WheelRPM& rpm);

  /// Transmit target RPMs to both boards with >= MIN_INTERFRAME_GAP_US spacing.
  void sendSpeeds();

  /// Zero all 4 wheel RPMs and transmit immediately.
  void stopAll();

  /// Check CAN link liveness for a single board (0=Rear 0x65, 1=Front 0x66).
  bool isBoardConnected(uint8_t boardIdx, uint32_t nowMs) const;

  /// Check CAN link liveness for all configured boards.
  bool isAllConnected(uint32_t nowMs) const;

  const MotorBoardState& boardState(uint8_t boardIdx) const;

  /// Throttled CAN link & RPM debug print.
  void printDebug(Print& out, uint32_t nowMs);

 private:
  void sendCanFrame(const CAN_message_t& msg);
  void handleRxFrame(const CAN_message_t& msg);
  static void onCanRxTrampoline(const CAN_message_t& msg);

  static MotorCanDriver* s_instance_;

  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_;
  MotorBoardState boards_[cfg::NUM_CAN_BOARDS];
  uint32_t lastTxUs_ = 0;
  bool hasSentFrame_ = false;
  uint32_t lastDebugMs_ = 0;
};
