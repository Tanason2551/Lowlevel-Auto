#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "motor_can.h"

struct MotorCanConfig {
  uint32_t canBaud = 1000000;
  uint8_t boardIdRear = 0x65;
  uint8_t boardIdFront = 0x66;
  float maxSafeRpm = 8000.0f;
  float gearRatio = 19.2f;
  int8_t wheelSign[4] = {-1, 1, -1, 1};  // FL, FR, RL, RR
  uint32_t interframeGapUs = 350;
  uint32_t rxTimeoutMs = 500;
  uint32_t debugPrintMs = 500;
};

/// Self-contained FlexCAN_T4 driver for 2 MTDriver Set-A boards (4 wheels).
class MotorCanDriver {
 public:
  static constexpr uint8_t NUM_BOARDS = 2;

  MotorCanDriver();

  void begin(const MotorCanConfig& cfg = MotorCanConfig{});
  void update();

  /// Apply gear ratio & wheel polarity, then stage RPMs for Rear(0) & Front(1).
  void setWheelRpms(float fl, float fr, float rl, float rr);

  /// Transmit target RPMs to both boards with >= interframeGapUs spacing.
  void sendSpeeds();

  /// Zero all 4 wheel RPMs and transmit immediately.
  void stopAll();

  /// Check CAN link liveness for a single board (0=Rear, 1=Front).
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

  MotorCanConfig cfg_{};
  MotorBoardState boards_[NUM_BOARDS];
  uint32_t lastTxUs_ = 0;
  bool hasSentFrame_ = false;
  uint32_t lastDebugMs_ = 0;
};
