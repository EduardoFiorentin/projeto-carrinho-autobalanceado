#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * Central project configuration.
 *
 * Keep hardware-independent control parameters here. Driver-specific pin maps
 * and actuator details should stay inside their own hardware abstraction class.
 */
namespace Config {
  // Main control loop period. 5000 us gives a nominal 200 Hz update rate.
  constexpr uint32_t CONTROL_PERIOD_US = 5000;

  // PID_v1 receives sample time in milliseconds, so this is derived from us.
  constexpr uint32_t CONTROL_PERIOD_MS = CONTROL_PERIOD_US / 1000;

  // Serial telemetry period. This is intentionally slower than the control loop.
  constexpr uint32_t TELEMETRY_PERIOD_US = 50000;

  // Signed controller command limits. The command sign selects motor direction.
  constexpr int MAX_POWER = 140;

  // Kept for future non-balance uses of PowerRamp; not used in the main loop.
  constexpr int MAX_STEP = 5;

  // Small command deadband around zero. Keep this separate from motor PWM minimums.
  constexpr double CONTROL_DEADBAND = 0.0;

  // Enable normal motor output after ARM/FALLEN/deadband protections are in place.
  constexpr bool MOTOR_OUTPUT_ENABLED = true;

  // Global closed-loop polarity. Flip this only if feedback is physically positive.
  constexpr int CONTROL_OUTPUT_SIGN = 1;

  // Robot only arms when close enough to the configured setpoint.
  constexpr float ARM_ANGLE_TOLERANCE_DEG = 5.0f;

  // Temporary actuation deadband around the setpoint to avoid high-PWM chatter.
  constexpr float ANGLE_CONTROL_DEADBAND_DEG = 1.0f;

  // Initial PID gains for angle control. Do not tune aggressively before sensor validation.
  constexpr double PID_KP = 0.8;
  constexpr double PID_KI = 0.0;
  constexpr double PID_KD = 0.0;

  // Desired vertical balance angle in degrees. Mechanical offset may require adjustment.
  constexpr double SETPOINT = 0.0;

  // Scalar Kalman parameters kept for experiments; not used in the main balance path.
  constexpr double KALMAN_Q = 0.0001;
  constexpr double KALMAN_R = 0.007;

  // Complementary filter coefficient. Higher alpha trusts gyro integration more.
  constexpr float COMPLEMENTARY_ALPHA = 0.98f;

  // Safety cutoff. Beyond this angle the robot is considered fallen.
  constexpr float MAX_SAFE_ANGLE_DEG = 35.0f;

  // Gyro bias calibration settings. Calibration runs before control is armed.
  constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 500;
  constexpr uint32_t SENSOR_SETTLE_DELAY_MS = 500;

  // MPU6050 I2C configuration.
  constexpr uint8_t MPU_ADDR = 0x68;
  constexpr int I2C_SDA_PIN = 21;
  constexpr int I2C_SCL_PIN = 22;
  constexpr uint32_t I2C_CLOCK = 100000;

  // Sensor full-scale selections written to the MPU6050 configuration registers.
  constexpr uint8_t ACCEL_RANGE_SEL = 2;
  constexpr uint8_t GYRO_RANGE_SEL = 2;

  // Physical mounting sign corrections for balance around the MPU6050 Y axis.
  constexpr float BALANCE_ACCEL_X_SIGN = 1.0f;
  constexpr float BALANCE_ACCEL_Z_SIGN = 1.0f;
  constexpr float BALANCE_ACCEL_ANGLE_SIGN = 1.0f;
  constexpr float BALANCE_GYRO_Y_SIGN = -1.0f;
}

#endif
