#include "SelfBalancingRobot.h"

#include <math.h>

#include "Config.h"

SelfBalancingRobot::SelfBalancingRobot()
  : angleFilter(Config::COMPLEMENTARY_ALPHA),
    controller(Config::PID_KP, Config::PID_KI, Config::PID_KD, Config::SETPOINT) {
}

void SelfBalancingRobot::begin() {
  // Serial is initialized here because all subsystems report boot/calibration status.
  Serial.begin(115200);
  delay(100);

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
  // Terminal safety states require a manual reset/reboot for now.
  if (state == RobotState::ERROR || state == RobotState::FALLEN) {
    motors.stop();
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

  // Read all converted axes for temporary physical-orientation diagnostics.
  float accelX = sensor.accelX();
  float accelY = sensor.accelY();
  float accelZ = sensor.accelZ();
  float gyroX = sensor.gyroX();
  float gyroY = sensor.gyroY();
  float gyroZ = sensor.gyroZ();

  // The balance path is intentionally unchanged during this diagnostic step.
  float accelAngle = sensor.accelAngle();
  float gyroRate = sensor.balanceGyroRate();

  // Complementary fusion: fast gyro response corrected by gravity reference.
  float estimatedAngle = angleFilter.update(
    accelAngle,
    gyroRate,
    dt
  );

  // Once the angle is beyond the recoverable envelope, stop immediately.
  if (fabs(estimatedAngle) > Config::MAX_SAFE_ANGLE_DEG) {
    enterFallen(estimatedAngle);
    if (nowUs - lastTelemetryUs >= Config::TELEMETRY_PERIOD_US) {
      lastTelemetryUs = nowUs;
      printTelemetry(nowUs, accelAngle, gyroRate, estimatedAngle, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    }
    return;
  }

  // The first valid cycle after calibration arms the balance loop.
  if (state == RobotState::READY) {
    state = RobotState::BALANCING;
  }

  // The PID input is the estimated angle in degrees. The output is a signed
  // command that already carries direction information.
  double controlOutput = controller.compute(estimatedAngle);
  double motorCommand = constrain(
    controlOutput,
    -static_cast<double>(Config::MAX_POWER),
    static_cast<double>(Config::MAX_POWER)
  );

  // Keep output disabled while validating Serial telemetry and sensor signs.
  if (Config::MOTOR_OUTPUT_ENABLED) {
    motors.apply(motorCommand);
  } else {
    motors.stop();
  }

  // Telemetry is rate-limited independently from the control loop so Serial I/O
  // does not determine controller timing.
  if (nowUs - lastTelemetryUs >= Config::TELEMETRY_PERIOD_US) {
    lastTelemetryUs = nowUs;
    printTelemetry(nowUs, accelAngle, gyroRate, estimatedAngle, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
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

void SelfBalancingRobot::printTelemetry(
  uint32_t timestamp,
  float accelAngle,
  float gyroRate,
  float estimatedAngle,
  float accelX,
  float accelY,
  float accelZ,
  float gyroX,
  float gyroY,
  float gyroZ
) {
  // Format:
  // timestamp_us, accelAngle_deg, gyroRate_dps, estimatedAngle_deg,
  // accelX_g, accelY_g, accelZ_g, gyroX_dps, gyroY_dps, gyroZ_dps
  Serial.printf(
    "%lu\t%.3f\t%.3f\t%.3f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%.3f\n",
    static_cast<unsigned long>(timestamp),
    accelAngle,
    gyroRate,
    estimatedAngle,
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ
  );
}
