#ifndef SELF_BALANCING_ROBOT_H
#define SELF_BALANCING_ROBOT_H

#include <Arduino.h>

#include "BalanceController.h"
#include "ComplementaryFilter.h"
#include "MotorDriver.h"
#include "Mpu6050Sensor.h"

/**
 * High-level application coordinator for the self-balancing robot.
 *
 * This class owns the real-time flow:
 * MPU6050 -> angle estimation -> PID -> signed motor command -> MotorDriver.
 * Hardware details remain inside the sensor and motor abstractions.
 */
class SelfBalancingRobot {
  public:
    SelfBalancingRobot();

    /**
     * Initializes serial output, motors, sensor, gyro calibration, PID, and state.
     * Motors remain stopped during initialization and calibration.
     */
    void begin();

    /**
     * Runs one scheduler pass. The method returns quickly until the next
     * configured control period has elapsed.
     */
    void update();

  private:
    /**
     * Minimal safety state machine.
     *
     * FALLEN and ERROR are terminal for now and require reset/reboot to rearm.
     */
    enum class RobotState {
      CALIBRATING,
      READY,
      BALANCING,
      FALLEN,
      ERROR
    };

    // Stop all actuation and move to ERROR with an observable serial message.
    void enterError(const char *reason);

    // Stop all actuation after exceeding the safe angle envelope.
    void enterFallen(float estimatedAngle);

    // Emit low-rate tab-separated data for Serial Plotter or external logging.
    void printTelemetry(
      uint32_t timestamp,
      float accelAngle,
      float gyroRate,
      float estimatedAngle,
      double controlOutput,
      double motorCommand
    );

    // Hardware and control collaborators.
    Mpu6050Sensor sensor;
    MotorDriver motors;
    ComplementaryFilter angleFilter;
    BalanceController controller;

    // Scheduler timestamps in microseconds. Unsigned subtraction tolerates rollover.
    uint32_t lastUpdateUs = 0;
    uint32_t lastTelemetryUs = 0;

    RobotState state = RobotState::CALIBRATING;
};

#endif
