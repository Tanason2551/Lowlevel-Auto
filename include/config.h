#pragma once

#include <stdint.h>

namespace cfg {

// ============================================================================
// Control Loop & Timing
// ============================================================================
constexpr uint32_t CONTROL_PERIOD_US   = 10000;  // 100 Hz control loop
constexpr uint32_t CAN_TX_INTERVAL_MS  = 20;     // 50 Hz CAN command TX
constexpr uint32_t CAN_RX_TIMEOUT_MS   = 500;    // CAN link liveness timeout
constexpr uint32_t DEBUG_PRINT_MS      = 500;    // Serial debug print period

// ============================================================================
// Mecanum X-Configuration Geometry & Motor Polarity
// Robot frame: +X forward, +Y left, +omega CCW
// ============================================================================
constexpr float WHEEL_RADIUS   = 0.050f;  // meters (100 mm wheel)
constexpr float HALF_LENGTH_X  = 0.180f;  // Lx: center to front/rear axle (m)
constexpr float HALF_WIDTH_Y   = 0.175f;  // Ly: center to left/right wheel (m)
constexpr float MAX_WHEEL_RPM  = 200.0f;  
constexpr float GEAR_RATIO     = 19.2f;   // Motor gearbox ratio (applied in MotorCanDriver)

// Index order: [0]=FL, [1]=FR, [2]=RL, [3]=RR
// Left wheels (FL, RL) are inverted (-1) to command negative RPM forward.
constexpr int8_t WHEEL_SIGN[4] = {-1, 1, -1, 1};

// ============================================================================
// Distance Control (Point-to-Point Phase 1: Distance only, omega = 0)
// ============================================================================
constexpr float POS_TOLERANCE_M   = 0.02f;  // Stop threshold (m)
constexpr float MAX_LINEAR_VEL    = 0.80f;  // Max Vr (m/s)
constexpr float MAX_LINEAR_ACCEL  = 1.50f;  // Max dVr/dt (m/s^2)

constexpr float DIST_KP = 1.60f;
constexpr float DIST_KI = 0.00f;
constexpr float DIST_KD = 0.05f;
constexpr float DIST_INTEGRAL_LIMIT = 0.30f;

// ============================================================================
// MTDriver CAN Bus Configuration (Ported from testcanmudule)
// Teensy 4.1 CAN1: TX=Pin 22, RX=Pin 23
// ============================================================================
constexpr uint32_t CAN_BAUD               = 1000000;  // 1 Mbps
constexpr uint8_t  NUM_CAN_BOARDS         = 2;
constexpr uint8_t  BOARD_ID_REAR          = 0x65;     // M1=RL, M2=RR
constexpr uint8_t  BOARD_ID_FRONT         = 0x66;     // M3=FL, M4=FR
constexpr float    MAX_SAFE_CAN_RPM       = 8000.0f;  // Hard safety clamp
constexpr float    RPM_TO_CMD_SCALE       = 1.0f;
constexpr uint32_t MIN_INTERFRAME_GAP_US  = 350;      // MTDriver spec >= 300 us

// ============================================================================
// PS4 Zigbee Joystick Configuration (Ported from testJoy-zigbee, Serial7)
// ============================================================================
constexpr uint32_t JOY_BAUD            = 115200;
constexpr uint32_t JOY_TIMEOUT_MS      = 400;   // Zero velocity if Zigbee stalls
constexpr int8_t   JOY_LX_OFFSET       = 30;
constexpr int8_t   JOY_LY_OFFSET       = 30;
constexpr int8_t   JOY_RX_OFFSET       = 30;
constexpr int8_t   JOY_RY_OFFSET       = 60;

// Axis signs (+1 or -1) to match robot frame (+X fwd, +Y left, +omega CCW)
constexpr int8_t   JOY_SIGN_VX         = 1;     // _Ly -> vx
constexpr int8_t   JOY_SIGN_VY         = -1;    // _Lx -> vy
constexpr int8_t   JOY_SIGN_OMEGA      = -1;    // _Rx -> omega

constexpr float    JOY_MAX_LINEAR_VEL  = 0.55f; // m/s (~2017 motor RPM @ 19.2:1)
constexpr float    JOY_MAX_ANGULAR_VEL = 1.50f; // rad/s
constexpr float    JOY_LINEAR_ACCEL    = 2.00f; // m/s^2 slew limit
constexpr float    JOY_ANGULAR_ACCEL   = 5.00f; // rad/s^2 slew limit

} 
