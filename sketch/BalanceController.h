#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

/**
 * Wrapper around PID_v1 for balance angle control.
 *
 * The controller input is the estimated robot angle in degrees. The output is a
 * signed command where the sign represents direction and the magnitude
 * represents requested motor effort.
 */
class BalanceController {
  public:
    BalanceController(double kp, double ki, double kd, double setpoint);

    // Configure PID mode, output range, and sample period.
    void begin(uint32_t sampleTimeMs, int maxPower);

    // Update the PID with the latest estimated angle and return the signed output.
    double compute(double inputValue);

    // Accessors used by telemetry and diagnostics.
    double output() const;
    double setpoint() const;

    // Runtime adjustment hooks for tuning.
    void setSetpoint(double value);
    void setTunings(double kp, double ki, double kd);

  private:
    // PID_v1 stores pointers to these variables, so their lifetime must match pid.
    double input = 0.0;
    double controlOutput = 0.0;
    double target = 0.0;
    PID pid;
};

#endif
