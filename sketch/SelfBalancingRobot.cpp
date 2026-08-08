#include "SelfBalancingRobot.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include "esp_system.h"

#include "Config.h"

namespace {
  const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
      case ESP_RST_POWERON: return "POWERON";
      case ESP_RST_EXT: return "EXT";
      case ESP_RST_SW: return "SW";
      case ESP_RST_PANIC: return "PANIC";
      case ESP_RST_INT_WDT: return "INT_WDT";
      case ESP_RST_TASK_WDT: return "TASK_WDT";
      case ESP_RST_WDT: return "WATCHDOG";
      case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
      case ESP_RST_BROWNOUT: return "BROWNOUT";
      case ESP_RST_SDIO: return "SDIO";
      case ESP_RST_UNKNOWN:
      default: return "UNKNOWN";
    }
  }
}

SelfBalancingRobot::SelfBalancingRobot()
  : angleFilter(Config::COMPLEMENTARY_ALPHA),
    controller(Config::PID_KP, Config::PID_KI, Config::PID_KD, Config::SETPOINT) {
}

void SelfBalancingRobot::begin() {
  // Serial is initialized here because all subsystems report boot/calibration status.
  Serial.begin(115200);
  delay(100);
  Serial.print("Reset reason: ");
  Serial.println(resetReasonName(esp_reset_reason()));

  // The robot starts disarmed. Motor outputs are explicitly disabled before any
  // sensor initialization or calibration work is attempted.
  state = RobotState::CALIBRATING;
  motors.begin();
  motors.stop();

  // Sensor initialization failures are safety-critical: without fresh IMU data,
  // the controller would be acting on stale or nonexistent state.
  if (!sensor.begin()) {
    enterError("MPU initialization failed");
    return;
  }

  // Give the sensor and mechanical structure a short settling window before
  // collecting gyro bias samples.
  delay(Config::SENSOR_SETTLE_DELAY_MS);

  // Gyro calibration assumes the robot is motionless. A failed calibration keeps
  // the system in ERROR instead of silently using zero bias.
  if (!sensor.calibrate(Config::GYRO_CALIBRATION_SAMPLES)) {
    enterError("Gyro calibration failed");
    return;
  }

  // PID_v1 uses a millisecond sample time. The main scheduler below still uses
  // micros() so the angle estimator can receive a real dt.
  controller.begin(Config::CONTROL_PERIOD_MS, Config::MAX_POWER);
  lastUpdateUs = micros();
  state = RobotState::READY;
}

void SelfBalancingRobot::update() {
  handleSerialDiagnostics();

  // Terminal safety states require a manual reset/reboot for now.
  if (state == RobotState::ERROR || state == RobotState::FALLEN) {
    motors.stop();
    return;
  }

  if (diagnosticStopRequested) {
    motors.stop();
    diagnosticStopRequested = false;
    return;
  }

  // Diagnostic motor commands intentionally bypass the sensor/PID path. Use
  // only with the wheels suspended, and send "S" to return to normal stopped mode.
  if (diagnosticMotorMode) {
    return;
  }

  // Non-blocking fixed-period scheduler. Unsigned subtraction keeps working
  // across micros() rollover.
  uint32_t nowUs = micros();
  uint32_t deltaUs = nowUs - lastUpdateUs;
  if (deltaUs < Config::CONTROL_PERIOD_US) {
    return;
  }
  lastUpdateUs = nowUs;

  // Use the actual elapsed time for gyro integration instead of assuming the
  // nominal control period was exact.
  float dt = deltaUs * 1.0e-6f;

  // Never continue the control cycle with stale sensor values.
  if (!sensor.read()) {
    enterError("MPU read failed");
    return;
  }

  float accelAngle = sensor.accelAngle();
  float gyroRate = sensor.balanceGyroRate();

  // Complementary fusion: fast gyro response corrected by gravity reference.
  float estimatedAngle = angleFilter.update(
    accelAngle,
    gyroRate,
    dt
  );

  float angleError = estimatedAngle - static_cast<float>(controller.setpoint());

  // Once the error is beyond the recoverable envelope, stop immediately.
  if (fabs(angleError) > Config::MAX_SAFE_ANGLE_DEG) {
    enterFallen(estimatedAngle);
    if (nowUs - lastTelemetryUs >= Config::TELEMETRY_PERIOD_US) {
      lastTelemetryUs = nowUs;
      printTelemetry(
        nowUs,
        stateName(),
        accelAngle,
        estimatedAngle,
        angleError,
        gyroRate,
        0.0,
        0.0,
        motors.rightAppliedDuty(),
        motors.leftAppliedDuty()
      );
    }
    return;
  }

  if (state == RobotState::READY) {
    if (fabs(angleError) > Config::ARM_ANGLE_TOLERANCE_DEG) {
      motors.stop();
      if (nowUs - lastTelemetryUs >= Config::TELEMETRY_PERIOD_US) {
        lastTelemetryUs = nowUs;
        printTelemetry(
          nowUs,
          stateName(),
          accelAngle,
          estimatedAngle,
          angleError,
          gyroRate,
          0.0,
          0.0,
          motors.rightAppliedDuty(),
          motors.leftAppliedDuty()
        );
      }
      return;
    }

    state = RobotState::BALANCING;
  }

  // The PID input is the estimated angle in degrees. The output is a signed
  // command that already carries direction information.
  double pidOutput = controller.compute(estimatedAngle);
  double motorCommand = constrain(
    Config::CONTROL_OUTPUT_SIGN * pidOutput,
    -static_cast<double>(Config::MAX_POWER),
    static_cast<double>(Config::MAX_POWER)
  );

  // The angle deadband affects only physical actuation, not the angle estimate
  // or the PID input.
  if (fabs(angleError) <= Config::ANGLE_CONTROL_DEADBAND_DEG) {
    motorCommand = 0.0;
  }

  if (Config::MOTOR_OUTPUT_ENABLED) {
    motors.apply(motorCommand);
  } else {
    motors.stop();
  }

  // Telemetry is rate-limited independently from the control loop so Serial I/O
  // does not determine controller timing.
  if (nowUs - lastTelemetryUs >= Config::TELEMETRY_PERIOD_US) {
    lastTelemetryUs = nowUs;
    printTelemetry(
      nowUs,
      stateName(),
      accelAngle,
      estimatedAngle,
      angleError,
      gyroRate,
      pidOutput,
      motorCommand,
      motors.rightAppliedDuty(),
      motors.leftAppliedDuty()
    );
  }
}

void SelfBalancingRobot::enterError(const char *reason) {
  // ERROR means the input data or initialization sequence is not trustworthy.
  motors.stop();
  state = RobotState::ERROR;
  Serial.print("ERROR: ");
  Serial.println(reason);
}

void SelfBalancingRobot::enterFallen(float estimatedAngle) {
  // FALLEN means the robot exceeded the safe angle envelope. Do not keep trying
  // to recover while the chassis is likely on the ground.
  motors.stop();
  state = RobotState::FALLEN;
  Serial.print("FALLEN. estimatedAngle: ");
  Serial.println(estimatedAngle);
}

const char *SelfBalancingRobot::stateName() const {
  switch (state) {
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::READY: return "READY";
    case RobotState::BALANCING: return "BALANCING";
    case RobotState::FALLEN: return "FALLEN";
    case RobotState::ERROR: return "ERROR";
  }

  return "UNKNOWN";
}

void SelfBalancingRobot::handleSerialDiagnostics() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      diagnosticCommandBuffer[diagnosticCommandLength] = '\0';
      processDiagnosticCommand(diagnosticCommandBuffer);
      diagnosticCommandLength = 0;
      diagnosticCommandBuffer[0] = '\0';
      continue;
    }

    if (diagnosticCommandLength < sizeof(diagnosticCommandBuffer) - 1) {
      diagnosticCommandBuffer[diagnosticCommandLength++] = c;
    } else {
      diagnosticCommandLength = 0;
      diagnosticCommandBuffer[0] = '\0';
      Serial.println("Diagnostic command too long");
    }
  }
}

void SelfBalancingRobot::processDiagnosticCommand(char *commandLine) {
  while (*commandLine == ' ' || *commandLine == '\t') {
    commandLine++;
  }

  if (*commandLine == '\0') {
    return;
  }

  char command = static_cast<char>(toupper(*commandLine));
  long value = strtol(commandLine + 1, nullptr, 10);

  if ((state == RobotState::ERROR || state == RobotState::FALLEN) && command != 'S') {
    motors.stop();
    Serial.println("Diagnostic command ignored in terminal safety state");
    return;
  }

  if (command == 'S') {
    motors.stop();
    diagnosticMotorMode = false;
    diagnosticStopRequested = true;
    Serial.println("DIAG STOP");
  } else if (command == 'R') {
    int rawDuty = constrain(static_cast<int>(value), -511, 511);
    motors.testRightRaw(rawDuty);
    diagnosticMotorMode = true;
    Serial.print("DIAG RIGHT RAW ");
    Serial.println(rawDuty);
  } else if (command == 'L') {
    int rawDuty = constrain(static_cast<int>(value), -511, 511);
    motors.testLeftRaw(rawDuty);
    diagnosticMotorMode = true;
    Serial.print("DIAG LEFT RAW ");
    Serial.println(rawDuty);
  } else if (command == 'B') {
    int logicalCommand = constrain(static_cast<int>(value), -Config::MAX_POWER, Config::MAX_POWER);
    if (logicalCommand == 0) {
      motors.stop();
      diagnosticMotorMode = false;
      diagnosticStopRequested = true;
    } else {
      motors.apply(logicalCommand);
      diagnosticMotorMode = true;
    }
    Serial.print("DIAG BOTH LOGICAL ");
    Serial.println(logicalCommand);
  } else {
    Serial.println("Unknown diagnostic command. Use R n, L n, B n, or S.");
  }
}

void SelfBalancingRobot::printTelemetry(
  uint32_t timestamp,
  const char *stateText,
  float accelAngle,
  float estimatedAngle,
  float angleError,
  float gyroRate,
  double pidOutput,
  double motorCommand,
  int rightAppliedDuty,
  int leftAppliedDuty
) {
  // Format:
  // timestamp_us, state, accelAngle_deg, estimatedAngle_deg, angleError_deg,
  // gyroRate_dps, pidOutput, motorCommand, rightAppliedDuty, leftAppliedDuty
  Serial.printf(
    "%lu\t%s\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%d\n",
    static_cast<unsigned long>(timestamp),
    stateText,
    accelAngle,
    estimatedAngle,
    angleError,
    gyroRate,
    pidOutput,
    motorCommand,
    rightAppliedDuty,
    leftAppliedDuty
  );
}
