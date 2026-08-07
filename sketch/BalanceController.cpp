#include "BalanceController.h"

BalanceController::BalanceController(double kp, double ki, double kd, double setpoint)
  : target(setpoint),
    pid(&input, &controlOutput, &target, kp, ki, kd, DIRECT) {
}

void BalanceController::begin(uint32_t sampleTimeMs, int maxPower) {
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-maxPower, maxPower);
  pid.SetSampleTime(sampleTimeMs);
}

double BalanceController::compute(double inputValue) {
  input = inputValue;
  pid.Compute();
  return controlOutput;
}

double BalanceController::output() const {
  return controlOutput;
}

void BalanceController::setSetpoint(double value) {
  target = value;
}

void BalanceController::setTunings(double kp, double ki, double kd) {
  pid.SetTunings(kp, ki, kd);
}
