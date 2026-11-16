#pragma once

#include <math.h>

namespace math_utils {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;

inline float degToRad(float deg) {
  return deg * (PI_F / 180.0f);
}

inline float radToDeg(float rad) {
  return rad * (180.0f / PI_F);
}


inline float wrapAngle(float rad) {
  float wrapped = fmodf(rad + PI_F, TWO_PI_F);
  if (wrapped < 0.0f) {
    wrapped += TWO_PI_F;
  }
  return wrapped - PI_F;
}

inline float clamp(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}


inline float slewRateLimit(float current, float target, float maxRate, float dt) {
  if (dt <= 0.0f || maxRate <= 0.0f) {
    return target;
  }
  const float maxDelta = maxRate * dt;
  const float delta = target - current;
  if (delta > maxDelta) return current + maxDelta;
  if (delta < -maxDelta) return current - maxDelta;
  return target;
}

} 
