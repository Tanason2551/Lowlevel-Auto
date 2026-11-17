#include "joy.h"

#include <stdlib.h>

JOY joy;

namespace {
uint8_t rxBuf[12] = {0};
uint8_t rxCount = 0;
}  // namespace

void JOY::init(const JoyConfig& cfg) {
  cfg_ = cfg;
  JOY_PORT.begin(cfg_.baud);
}

bool JOY::parsePacket(const uint8_t* buf, uint32_t nowMs) {
  if (buf == nullptr || buf[0] != '#' || buf[1] != 's' || buf[10] != '\r' ||
      buf[11] != '\n') {
    return false;
  }

  triangle = (buf[2] >> 7) & 0x01;
  circle = (buf[2] >> 6) & 0x01;
  cross = (buf[2] >> 5) & 0x01;
  square = (buf[2] >> 4) & 0x01;
  up = (buf[2] >> 3) & 0x01;
  right = (buf[2] >> 2) & 0x01;
  down = (buf[2] >> 1) & 0x01;
  left = buf[2] & 0x01;

  share = (buf[3] >> 7) & 0x01;
  option = (buf[3] >> 6) & 0x01;
  touchpad = (buf[3] >> 5) & 0x01;
  ps = (buf[3] >> 4) & 0x01;
  L3 = (buf[3] >> 3) & 0x01;
  R3 = (buf[3] >> 2) & 0x01;
  L1 = (buf[3] >> 1) & 0x01;
  R1 = buf[3] & 0x01;

  L2 = buf[4];
  R2 = buf[5];

  Lx = buf[6];
  _Lx = static_cast<int8_t>(map(Lx, 0, 255, -127, 127));
  Ly = buf[7];
  _Ly = static_cast<int8_t>(map(Ly, 0, 255, -127, 127));
  Rx = buf[8];
  _Rx = static_cast<int8_t>(map(Rx, 0, 255, -127, 127));
  Ry = buf[9];
  _Ry = static_cast<int8_t>(map(Ry, 0, 255, -127, 127));

  if (abs(_Lx) < cfg_.lxOffset) _Lx = 0;
  if (abs(_Ly) < cfg_.lyOffset) _Ly = 0;
  if (abs(_Rx) < cfg_.rxOffset) _Rx = 0;
  if (abs(_Ry) < cfg_.ryOffset) _Ry = 0;

  count = nowMs;
  flip_bit = !flip_bit;
  isConnected = true;
  return true;
}

bool JOY::isLinkAlive(uint32_t nowMs) const {
  return isConnected && ((nowMs - count) <= cfg_.timeoutMs);
}

void serialEvent7(void) {
  while (JOY_PORT.available() > 0) {
    const uint8_t inByte = static_cast<uint8_t>(JOY_PORT.read());

    if (rxCount == 0) {
      if (inByte == '#') {
        rxBuf[rxCount++] = inByte;
      }
      continue;
    }

    if (rxCount == 1) {
      if (inByte == 's') {
        rxBuf[rxCount++] = inByte;
      } else if (inByte == '#') {
        rxBuf[0] = '#';
        rxCount = 1;
      } else {
        rxCount = 0;
      }
      continue;
    }

    rxBuf[rxCount++] = inByte;
    if (rxCount >= 12) {
      joy.parsePacket(rxBuf, millis());
      rxCount = 0;
    }
  }
}

void JOY::serialTransmitter() {
  txBuffer[0] = '#';
  txBuffer[1] = 's';

  int8t_converter.asInt = 0;
  txBuffer[2] = int8t_converter.asByte;
  int8t_converter.asInt = static_cast<int8_t>(display);
  txBuffer[3] = int8t_converter.asByte;

  txBuffer[4] = '\r';
  txBuffer[5] = '\n';

  JOY_PORT.write(txBuffer, sizeof(txBuffer));
}

void JOY::debug() const {
  Serial.print(F("map_Lx="));
  Serial.print(_Lx);
  Serial.print(F(" map_Ly="));
  Serial.print(_Ly);
  Serial.print(F(" map_Rx="));
  Serial.print(_Rx);
  Serial.print(F(" map_Ry="));
  Serial.println(_Ry);
}
