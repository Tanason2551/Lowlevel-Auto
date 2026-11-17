#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef INT8_CONVERTOR_DEFINED
#define INT8_CONVERTOR_DEFINED
typedef union {
  int8_t asInt;
  uint8_t asByte;
} INT8_CONVERTOR;
#endif

#define JOY_PORT Serial7
#define JOY_TX_SIZE 6

struct JOY {
  // Raw stick bytes (0..255)
  uint8_t Lx = 0;
  uint8_t Ly = 0;
  uint8_t Rx = 0;
  uint8_t Ry = 0;

  // Mapped & deadzone-filtered stick values (-127..127)
  int8_t _Lx = 0;
  int8_t _Ly = 0;
  int8_t _Rx = 0;
  int8_t _Ry = 0;

  bool L1 = false;
  bool R1 = false;
  uint8_t L2 = 0;
  uint8_t R2 = 0;
  bool L3 = false;
  bool R3 = false;

  bool triangle = false;
  bool circle = false;
  bool cross = false;
  bool square = false;
  bool option = false;

  bool up = false;
  bool right = false;
  bool down = false;
  bool left = false;
  bool share = false;

  bool isConnected = false;
  bool ps = false;
  bool touchpad = false;

  uint32_t count = 0;  // Timestamp (ms) of last valid RX frame
  bool flip_bit = false;

  INT8_CONVERTOR int8t_converter{};
  uint8_t txBuffer[JOY_TX_SIZE] = {0};
  uint8_t display = 0;

  void init();
  bool parsePacket(const uint8_t* buf, uint32_t nowMs);
  bool isLinkAlive(uint32_t nowMs) const;
  void debug() const;
  void serialTransmitter();
};

extern JOY joy;
