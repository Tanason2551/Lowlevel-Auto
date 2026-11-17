#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <FlexCAN_T4.h>
#else
// Lightweight host-test shim so CAN frame packing/parsing can be unit-tested.
struct CAN_message_t {
  uint32_t id = 0;
  uint8_t len = 0;
  struct {
    uint8_t extended = 0;
  } flags;
  uint8_t buf[8] = {0};
};
#endif

// ============================================================================
// MTDriver Set A (Wheel Driver) Protocol: 2 Boards x 2 Motors = 4 Motors
// Self-contained header (no dependency on project include/config.h).
// ============================================================================
constexpr float MT_DEFAULT_MAX_SAFE_RPM = 8000.0f;
constexpr float MT_DEFAULT_RPM_SCALE = 1.0f;

typedef enum : uint8_t {
  MT_CMD_SET_CURRENT_SPEED = 0x02,
  MT_CMD_MOTOR_STATUS = 0x08,
} MT_CMD_SET_A_T;

union FloatBytes {
  float asFloat;
  uint8_t asUint8_t[4];
};

struct MotorBoardState {
  uint8_t node_id = 0;
  float target_rpm[2] = {0.0f, 0.0f};
  float position = 0.0f;
  float current_speed = 0.0f;
  float current_amp = 0.0f;
  uint32_t last_rx_ms = 0;
  uint32_t rx_count = 0;
};

inline float clampRpmValue(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

inline float sanitizeRpm(float rpm, float maxRpm, float scale) {
  if (!isfinite(rpm)) {
    return 0.0f;
  }
  return clampRpmValue(rpm, -maxRpm, maxRpm) * scale;
}

inline void packFloatBE(uint8_t* dst, float value) {
  FloatBytes f2b;
  f2b.asFloat = value;
  dst[0] = f2b.asUint8_t[3];
  dst[1] = f2b.asUint8_t[2];
  dst[2] = f2b.asUint8_t[1];
  dst[3] = f2b.asUint8_t[0];
}

inline float unpackFloatBE(const uint8_t* src) {
  FloatBytes f2b;
  f2b.asUint8_t[3] = src[0];
  f2b.asUint8_t[2] = src[1];
  f2b.asUint8_t[1] = src[2];
  f2b.asUint8_t[0] = src[3];
  return f2b.asFloat;
}

inline uint16_t unpackU16BE(const uint8_t* src) {
  return (static_cast<uint16_t>(src[0]) << 8) | static_cast<uint16_t>(src[1]);
}

inline int16_t unpackI16BE(const uint8_t* src) {
  return static_cast<int16_t>(unpackU16BE(src));
}

inline CAN_message_t makeExtendedFrame(uint8_t id, MT_CMD_SET_A_T cmd,
                                       uint8_t len) {
  CAN_message_t msg;
  msg.id = static_cast<uint32_t>(id) | (static_cast<uint32_t>(cmd) << 8);
  msg.len = len;
  msg.flags.extended = 1;
  return msg;
}

/// Build an 8-byte SET_CURRENT_SPEED (0x02) frame for a 2-motor board.
inline CAN_message_t buildSetCurrentSpeed2(
    uint8_t id, float rpm1, float rpm2,
    float maxRpm = MT_DEFAULT_MAX_SAFE_RPM,
    float scale = MT_DEFAULT_RPM_SCALE) {
  CAN_message_t msg = makeExtendedFrame(id, MT_CMD_SET_CURRENT_SPEED, 8);
  packFloatBE(msg.buf, sanitizeRpm(rpm1, maxRpm, scale));
  packFloatBE(msg.buf + 4, sanitizeRpm(rpm2, maxRpm, scale));
  return msg;
}

/// Parse MT_CMD_MOTOR_STATUS (0x08, 8 bytes) and update matching board state.
inline bool parseMotorStatus(const CAN_message_t& msg, MotorBoardState* boards,
                             size_t numBoards, uint32_t nowMs) {
  if (!msg.flags.extended || msg.len < 8 || boards == nullptr) {
    return false;
  }

  const uint8_t id = static_cast<uint8_t>(msg.id & 0xFF);
  const uint8_t cmd = static_cast<uint8_t>((msg.id >> 8) & 0xFF);

  if (cmd != MT_CMD_MOTOR_STATUS) {
    return false;
  }

  for (size_t i = 0; i < numBoards; i++) {
    if (boards[i].node_id == id) {
      boards[i].position = static_cast<float>(unpackU16BE(msg.buf)) / 100.0f;
      boards[i].current_speed = unpackFloatBE(msg.buf + 2);
      boards[i].current_amp =
          static_cast<float>(unpackI16BE(msg.buf + 6)) / 100.0f;
      boards[i].last_rx_ms = nowMs;
      boards[i].rx_count++;
      return true;
    }
  }

  return false;
}
