#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include <joy.h>
#include <motor_can_driver.h>
#include <ros1_serial_rx.h>

#include "config.h"
#include "distance_control.h"
#include "joy_drive.h"
#include "kinematics.h"
#include "math_utils.h"

// ============================================================================
// Core Modules (Separated into dedicated files)
// ============================================================================
static DistanceController distanceCtrl(cfg::DIST_KP, cfg::DIST_KI, cfg::DIST_KD,
                                       cfg::DIST_INTEGRAL_LIMIT,
                                       cfg::MAX_LINEAR_VEL,
                                       cfg::MAX_LINEAR_ACCEL,
                                       cfg::POS_TOLERANCE_M);

static JoyDriveController joyDrive(cfg::JOY_MAX_LINEAR_VEL,
                                   cfg::JOY_MAX_ANGULAR_VEL,
                                   cfg::JOY_LINEAR_ACCEL,
                                   cfg::JOY_ANGULAR_ACCEL, cfg::JOY_SIGN_VX,
                                   cfg::JOY_SIGN_VY, cfg::JOY_SIGN_OMEGA);

static MecanumKinematics kinematics(cfg::WHEEL_RADIUS, cfg::HALF_LENGTH_X,
                                    cfg::HALF_WIDTH_Y, cfg::MAX_WHEEL_RPM);

static MotorCanDriver canMotor;
static Ros1SerialRx rosRx;

enum class DriveMode { JOY, DISTANCE };
static DriveMode driveMode = DriveMode::JOY;

// ============================================================================
// Pose & Goal Input Layer (High-Level ROS1 Serial1 or Stand-alone Simulation)
// ============================================================================
struct TestInputState {
  Pose2D pose{};
  Goal2D goal{};
  bool motionEnabled = false;
  bool useHighLevelPose = true;  // true = Serial1 ROS1 pose, false = sim pose
  bool simIntegrate = true;
};

static TestInputState inputState;

static void setDriveMode(DriveMode nextMode) {
  if (driveMode == nextMode) {
    return;
  }
  driveMode = nextMode;
  inputState.motionEnabled = false;
  distanceCtrl.reset();
  joyDrive.reset();
  canMotor.stopAll();
  Serial.print(F("[MODE] Switched to "));
  Serial.println(driveMode == DriveMode::JOY ? F("JOY") : F("DISTANCE"));
}

static void printTestHelp() {
  Serial.println(F("\n=== Mecanum Joy + Distance Control Console ==="));
  Serial.println(F("  mode joy / mode dist : Switch control mode (default: JOY)"));
  Serial.println(F("  g <x> <y>            : Set goal (m) & enable motion (DIST)"));
  Serial.println(F("  hl <0|1>             : Pose source 1=ROS1 Serial1, 0=Sim"));
  Serial.println(F("  p <x> <y> <yaw>      : Set simulated pose (when hl=0)"));
  Serial.println(F("  sim <0|1>            : Toggle simulated pose integration"));
  Serial.println(F("  0 / stop             : Stop all motors immediately"));
  Serial.println(F("  h                    : Show this help"));
}

static void handleTestSerialCommand(char* line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
    printTestHelp();
  } else if (strcmp(line, "0") == 0 || strcasecmp(line, "stop") == 0) {
    inputState.motionEnabled = false;
    distanceCtrl.reset();
    joyDrive.reset();
    canMotor.stopAll();
    Serial.println(F("[CMD] Stop -> 0 RPM sent"));
  } else if (strcasecmp(line, "mode joy") == 0) {
    setDriveMode(DriveMode::JOY);
  } else if (strcasecmp(line, "mode dist") == 0) {
    setDriveMode(DriveMode::DISTANCE);
  } else if (strncmp(line, "hl ", 3) == 0) {
    inputState.useHighLevelPose = (atoi(line + 3) != 0);
    distanceCtrl.reset();
    Serial.print(F("[CMD] Pose source: "));
    Serial.println(inputState.useHighLevelPose ? F("ROS1 Serial1") : F("SIM"));
  } else if (strncmp(line, "g ", 2) == 0) {
    char* p1 = nullptr;
    char* p2 = nullptr;
    const float gx = strtof(line + 2, &p1);
    const float gy = (p1 != line + 2) ? strtof(p1, &p2) : 0.0f;
    if (p1 == line + 2 || p2 == p1) {
      Serial.println(F("[ERR] Usage: g <x> <y>"));
      return;
    }
    inputState.goal.x = gx;
    inputState.goal.y = gy;
    inputState.motionEnabled = true;
    Serial.print(F("[CMD] Goal=("));
    Serial.print(gx, 2);
    Serial.print(',');
    Serial.print(gy, 2);
    Serial.println(F(") motion=ON"));
  } else if (strncmp(line, "p ", 2) == 0) {
    char* p1 = nullptr;
    char* p2 = nullptr;
    char* p3 = nullptr;
    const float px = strtof(line + 2, &p1);
    const float py = (p1 != line + 2) ? strtof(p1, &p2) : 0.0f;
    const float pyawDeg = (p2 != p1) ? strtof(p2, &p3) : 0.0f;
    if (p1 == line + 2 || p2 == p1 || p3 == p2) {
      Serial.println(F("[ERR] Usage: p <x> <y> <yaw_deg>"));
      return;
    }
    inputState.pose.x = px;
    inputState.pose.y = py;
    inputState.pose.yawRad =
        math_utils::wrapAngle(math_utils::degToRad(pyawDeg));
    Serial.print(F("[CMD] Pose=("));
    Serial.print(px, 2);
    Serial.print(',');
    Serial.print(py, 2);
    Serial.print(F(", "));
    Serial.print(pyawDeg, 1);
    Serial.println(F(" deg)"));
  } else if (strncmp(line, "sim ", 4) == 0) {
    inputState.simIntegrate = (atoi(line + 4) != 0);
    Serial.print(F("[CMD] Sim pose integration: "));
    Serial.println(inputState.simIntegrate ? F("ON") : F("OFF"));
  } else {
    Serial.print(F("[ERR] Unknown cmd: "));
    Serial.println(line);
  }
}

static void pollTestSerial() {
  static char buf[64];
  static size_t idx = 0;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (idx > 0) {
        buf[idx] = '\0';
        handleTestSerialCommand(buf);
        idx = 0;
      }
    } else if (idx + 1 < sizeof(buf)) {
      buf[idx++] = c;
    }
  }
}

static void initPoseAndGoalSource() {
  inputState = TestInputState{};
  rosRx.begin(Serial1, cfg::HL_SERIAL_BAUD);
  printTestHelp();
}

static void updatePoseAndGoal(const RobotVelocity& lastVel, float dt,
                              uint32_t nowMs, Pose2D& poseOut, Goal2D& goalOut,
                              bool& enabledOut) {
  if (inputState.useHighLevelPose) {
    const bool hlAlive = rosRx.isLinkAlive(nowMs, cfg::HL_TIMEOUT_MS);
    if (hlAlive) {
      float xM = 0.0f;
      float yM = 0.0f;
      float yawRad = 0.0f;
      rosRx.robotPose(xM, yM, yawRad);
      inputState.pose.x = xM;
      inputState.pose.y = yM;
      inputState.pose.yawRad = math_utils::wrapAngle(yawRad);
    }
    poseOut = inputState.pose;
    goalOut = inputState.goal;
    enabledOut = inputState.motionEnabled && hlAlive;
    return;
  }

  if (inputState.motionEnabled && inputState.simIntegrate && dt > 0.0f) {
    const float c = cosf(inputState.pose.yawRad);
    const float s = sinf(inputState.pose.yawRad);
    inputState.pose.x += (lastVel.vx * c - lastVel.vy * s) * dt;
    inputState.pose.y += (lastVel.vx * s + lastVel.vy * c) * dt;
  }

  poseOut = inputState.pose;
  goalOut = inputState.goal;
  enabledOut = inputState.motionEnabled;
}

static void handleJoyModeButtons(uint32_t nowMs) {
  static bool prevOption = false;
  const bool alive = joy.isLinkAlive(nowMs);
  const bool optionNow = alive && joy.option;

  if (optionNow && !prevOption) {
    setDriveMode(driveMode == DriveMode::JOY ? DriveMode::DISTANCE
                                             : DriveMode::JOY);
  }
  prevOption = optionNow;

  if (alive && joy.cross && driveMode == DriveMode::DISTANCE &&
      inputState.motionEnabled) {
    inputState.motionEnabled = false;
    distanceCtrl.reset();
    canMotor.stopAll();
    Serial.println(F("[JOY] Cross pressed -> DISTANCE stopped"));
  }
}

// ============================================================================
// Setup & 100 Hz Control Loop
// ============================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  const uint32_t startMs = millis();
  while (!Serial && (millis() - startMs < 1500)) {
  }

  if (CrashReport) {
    Serial.print(CrashReport);
  }
  Serial.println(F("[BOOT] Serial ready, initializing drivers..."));

  JoyConfig joyCfg;
  joyCfg.baud = cfg::JOY_BAUD;
  joyCfg.timeoutMs = cfg::JOY_TIMEOUT_MS;
  joyCfg.lxOffset = cfg::JOY_LX_OFFSET;
  joyCfg.lyOffset = cfg::JOY_LY_OFFSET;
  joyCfg.rxOffset = cfg::JOY_RX_OFFSET;
  joyCfg.ryOffset = cfg::JOY_RY_OFFSET;
  joy.init(joyCfg);

  MotorCanConfig canCfg;
  canCfg.canBaud = cfg::CAN_BAUD;
  canCfg.boardIdRear = cfg::BOARD_ID_REAR;
  canCfg.boardIdFront = cfg::BOARD_ID_FRONT;
  canCfg.maxSafeRpm = cfg::MAX_SAFE_CAN_RPM;
  canCfg.gearRatio = cfg::GEAR_RATIO;
  for (uint8_t i = 0; i < 4; i++) {
    canCfg.wheelSign[i] = cfg::WHEEL_SIGN[i];
  }
  canCfg.interframeGapUs = cfg::MIN_INTERFRAME_GAP_US;
  canCfg.rxTimeoutMs = cfg::CAN_RX_TIMEOUT_MS;
  canCfg.debugPrintMs = cfg::DEBUG_PRINT_MS;
  canMotor.begin(canCfg);

  initPoseAndGoalSource();
  Serial.println(F("[BOOT] Setup complete"));
}

void loop() {
  canMotor.update();
  rosRx.update();
  pollTestSerial();

  static uint32_t lastControlUs = micros();
  static uint32_t lastCanTxMs = 0;
  static uint32_t lastDebugMs = 0;
  static RobotVelocity lastVel{};
  static DistanceControlOutput lastDistOut{};
  static WheelRPM lastRpm{};

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  handleJoyModeButtons(nowMs);

  if (nowUs - lastControlUs >= cfg::CONTROL_PERIOD_US) {
    const float dt = static_cast<float>(nowUs - lastControlUs) * 1e-6f;
    lastControlUs = nowUs;

    if (driveMode == DriveMode::JOY) {
      const JoyAxesInput joyIn{joy._Lx, joy._Ly, joy._Ry,
                               joy.isLinkAlive(nowMs)};
      lastVel = joyDrive.compute(joyIn, dt);
      lastRpm = kinematics.inverseKinematics(lastVel);
    } else {
      Pose2D pose;
      Goal2D goal;
      bool motionEnabled = false;
      updatePoseAndGoal(lastVel, dt, nowMs, pose, goal, motionEnabled);

      if (!motionEnabled) {
        lastVel = RobotVelocity{};
        lastDistOut = DistanceControlOutput{};
        lastRpm = WheelRPM{};
        distanceCtrl.reset();
      } else {
        lastDistOut = distanceCtrl.compute(pose, goal, dt);
        lastVel =
            kinematics.movingSystem(lastDistOut.vr, lastDistOut.phiRad, 0.0f);
        lastRpm = kinematics.inverseKinematics(lastVel);
      }
    }

    canMotor.setWheelRpms(lastRpm.fl, lastRpm.fr, lastRpm.rl, lastRpm.rr);
  }

  if (nowMs - lastCanTxMs >= cfg::CAN_TX_INTERVAL_MS) {
    lastCanTxMs = nowMs;
    canMotor.sendSpeeds();
  }

  if (nowMs - lastDebugMs >= cfg::DEBUG_PRINT_MS) {
    lastDebugMs = nowMs;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (driveMode == DriveMode::JOY) {
      Serial.print(F("[JOY] link="));
      Serial.print(joy.isLinkAlive(nowMs) ? F("OK") : F("LOST"));
      Serial.print(F(" vx="));
      Serial.print(lastVel.vx, 2);
      Serial.print(F(" vy="));
      Serial.print(lastVel.vy, 2);
      Serial.print(F(" w="));
      Serial.print(lastVel.omega, 2);
      Serial.print(' ');
      Serial.print(F(" pose=("));
      Serial.print(inputState.pose.x, 2);
      Serial.print(',');
      Serial.print(inputState.pose.y, 2);
      Serial.print(',');
      Serial.print(math_utils::radToDeg(inputState.pose.yawRad), 1);
      Serial.println(F(")"));
      
    } else {
      Serial.print(F("[DIST] hl="));
      if (!inputState.useHighLevelPose) {
        Serial.print(F("SIM"));
      } else {
        Serial.print(rosRx.isLinkAlive(nowMs, cfg::HL_TIMEOUT_MS) ? F("OK")
                                                                  : F("LOST"));
      }
      Serial.print(F(" en="));
      Serial.print(inputState.motionEnabled ? 1 : 0);
      Serial.print(F(" pose=("));
      Serial.print(inputState.pose.x, 2);
      Serial.print(',');
      Serial.print(inputState.pose.y, 2);
      Serial.print(',');
      Serial.print(math_utils::radToDeg(inputState.pose.yawRad), 1);
      Serial.print(F(") S="));
      Serial.print(lastDistOut.distanceError, 3);
      Serial.print(F("m phi="));
      Serial.print(math_utils::radToDeg(lastDistOut.phiRad), 1);
      Serial.print(F("deg Vr="));
      Serial.print(lastDistOut.vr, 2);
      Serial.print(F(" goal="));
      Serial.println(lastDistOut.atGoal ? F("YES ") : F("NO "));
    }
    // canMotor.printDebug(Serial, nowMs);
  }
}
