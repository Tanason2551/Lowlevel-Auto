#include "drivers/motor_can_driver.h"

MotorCanDriver* MotorCanDriver::s_instance_ = nullptr;

MotorCanDriver::MotorCanDriver() {
  boards_[0] = MotorBoardState{};
  boards_[0].node_id = cfg::BOARD_ID_REAR;   // 0x65: M1=RL, M2=RR
  boards_[1] = MotorBoardState{};
  boards_[1].node_id = cfg::BOARD_ID_FRONT;  // 0x66: M3=FL, M4=FR
}

void MotorCanDriver::begin() {
  s_instance_ = this;

  can_.begin();
  can_.setBaudRate(cfg::CAN_BAUD);
  can_.enableFIFO();
  can_.enableFIFOInterrupt();
  can_.onReceive(onCanRxTrampoline);
  can_.events();
}

void MotorCanDriver::update() {
  can_.events();
}

void MotorCanDriver::setWheelRpms(const WheelRPM& rpm) {
  const float flCmd = sanitizeRpm(rpm.fl * cfg::WHEEL_SIGN[0],
                                  cfg::MAX_SAFE_CAN_RPM, 1.0f);
  const float frCmd = sanitizeRpm(rpm.fr * cfg::WHEEL_SIGN[1],
                                  cfg::MAX_SAFE_CAN_RPM, 1.0f);
  const float rlCmd = sanitizeRpm(rpm.rl * cfg::WHEEL_SIGN[2],
                                  cfg::MAX_SAFE_CAN_RPM, 1.0f);
  const float rrCmd = sanitizeRpm(rpm.rr * cfg::WHEEL_SIGN[3],
                                  cfg::MAX_SAFE_CAN_RPM, 1.0f);

  // Board 0 (0x65): Rear Left (M1), Rear Right (M2)
  boards_[0].target_rpm[0] = rlCmd;
  boards_[0].target_rpm[1] = rrCmd;

  // Board 1 (0x66): Front Left (M3), Front Right (M4)
  boards_[1].target_rpm[0] = flCmd;
  boards_[1].target_rpm[1] = frCmd;
}

void MotorCanDriver::sendCanFrame(const CAN_message_t& msg) {
  if (hasSentFrame_) {
    const uint32_t elapsedUs = micros() - lastTxUs_;
    if (elapsedUs < cfg::MIN_INTERFRAME_GAP_US) {
      delayMicroseconds(cfg::MIN_INTERFRAME_GAP_US - elapsedUs);
    }
  }

  can_.write(msg);
  lastTxUs_ = micros();
  hasSentFrame_ = true;
}

void MotorCanDriver::sendSpeeds() {
  for (uint8_t i = 0; i < cfg::NUM_CAN_BOARDS; i++) {
    const CAN_message_t msg = buildSetCurrentSpeed2(
        boards_[i].node_id, boards_[i].target_rpm[0], boards_[i].target_rpm[1]);
    sendCanFrame(msg);
  }
}

void MotorCanDriver::stopAll() {
  setWheelRpms(WheelRPM{0.0f, 0.0f, 0.0f, 0.0f});
  sendSpeeds();
}

bool MotorCanDriver::isBoardConnected(uint8_t boardIdx, uint32_t nowMs) const {
  if (boardIdx >= cfg::NUM_CAN_BOARDS) {
    return false;
  }
  const MotorBoardState& b = boards_[boardIdx];
  return (b.rx_count > 0) && ((nowMs - b.last_rx_ms) <= cfg::CAN_RX_TIMEOUT_MS);
}

bool MotorCanDriver::isAllConnected(uint32_t nowMs) const {
  for (uint8_t i = 0; i < cfg::NUM_CAN_BOARDS; i++) {
    if (!isBoardConnected(i, nowMs)) {
      return false;
    }
  }
  return true;
}

const MotorBoardState& MotorCanDriver::boardState(uint8_t boardIdx) const {
  const uint8_t safeIdx =
      (boardIdx < cfg::NUM_CAN_BOARDS) ? boardIdx : 0;
  return boards_[safeIdx];
}

void MotorCanDriver::printDebug(Print& out, uint32_t nowMs) {
  if (nowMs - lastDebugMs_ < cfg::DEBUG_PRINT_MS) {
    return;
  }
  lastDebugMs_ = nowMs;

  const bool ok = isAllConnected(nowMs);
  out.print(ok ? F("[CAN OK] ") : F("[CAN LOST] "));
  out.print(F("cmd[FL,FR,RL,RR]=["));
  out.print(boards_[1].target_rpm[0], 1);
  out.print(',');
  out.print(boards_[1].target_rpm[1], 1);
  out.print(',');
  out.print(boards_[0].target_rpm[0], 1);
  out.print(',');
  out.print(boards_[0].target_rpm[1], 1);
  out.print(F("]"));

  for (uint8_t i = 0; i < cfg::NUM_CAN_BOARDS; i++) {
    out.print(F(" | 0x"));
    out.print(boards_[i].node_id, HEX);
    out.print(isBoardConnected(i, nowMs) ? F(":OK") : F(":LOST"));
    out.print(F(" spd="));
    out.print(boards_[i].current_speed, 1);
    out.print(F(" rx="));
    out.print(boards_[i].rx_count);
  }
  out.println();
}

void MotorCanDriver::handleRxFrame(const CAN_message_t& msg) {
  parseMotorStatus(msg, boards_, cfg::NUM_CAN_BOARDS, millis());
}

void MotorCanDriver::onCanRxTrampoline(const CAN_message_t& msg) {
  if (s_instance_ != nullptr) {
    s_instance_->handleRxFrame(msg);
  }
}
