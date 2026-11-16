#pragma once

struct RobotVelocity {
  float vx = 0.0f;     // Forward velocity in robot frame (m/s)
  float vy = 0.0f;     // Leftward strafe velocity in robot frame (m/s)
  float omega = 0.0f;  // CCW angular velocity (rad/s)
};

struct WheelRPM {
  float fl = 0.0f;  // Front-Left (before motor polarity inversion)
  float fr = 0.0f;  // Front-Right
  float rl = 0.0f;  // Rear-Left
  float rr = 0.0f;  // Rear-Right
};


class MecanumKinematics {
 public:
  MecanumKinematics(float wheelRadiusM, float halfLengthXM, float halfWidthYM,
                    float maxWheelRpm);

  /// Convert polar speed (Vr, phi) + omega into robot-frame (Vx, Vy, omega).
  RobotVelocity movingSystem(float vr, float phiRad, float omega = 0.0f) const;

  /// Compute 4-wheel RPMs with proportional desaturation when exceeding max RPM.
  WheelRPM inverseKinematics(const RobotVelocity& vel) const;

 private:
  float wheelRadiusM_;
  float halfLengthXM_;
  float halfWidthYM_;
  float maxWheelRpm_;
};
