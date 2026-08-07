#include "BalanceController.h"

BalanceController::BalanceController(double kp, double ki, double kd, double setpoint)
  : target(setpoint),
    // PID_v1 works by holding pointers to input, output, and setpoint storage.
    pid(&input, &controlOutput, &target, kp, ki, kd, DIRECT) {
}

void BalanceController::begin(uint32_t sampleTimeMs, int maxPower) {
  // Limit the signed output so the MotorDriver receives a bounded command.
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-maxPower, maxPower);

  // This sample time should match the nominal scheduler period.
  pid.SetSampleTime(sampleTimeMs);
}

double BalanceController::compute(double inputValue) {
  // The caller supplies estimated angle in degrees. PID_v1 writes the output
  // into controlOutput through the pointer configured in the constructor.
  input = inputValue;
  pid.Compute();
  return controlOutput;
}

double BalanceController::output() const {
  return controlOutput;
}

double BalanceController::setpoint() const {
  return target;
}

void BalanceController::setSetpoint(double value) {
  // The setpoint may need a small mechanical offset after physical validation.
  target = value;
}

void BalanceController::setTunings(double kp, double ki, double kd) {
  // Runtime tuning hook used during bench testing.
  pid.SetTunings(kp, ki, kd);
}
