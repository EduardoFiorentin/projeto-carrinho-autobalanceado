#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

class BalanceController {
  public:
    BalanceController(double kp, double ki, double kd, double setpoint);

    void begin(uint32_t sampleTimeMs, int maxPower);
    double compute(double inputValue);
    double output() const;
    void setSetpoint(double value);
    void setTunings(double kp, double ki, double kd);

  private:
    double input = 0.0;
    double controlOutput = 0.0;
    double target = 0.0;
    PID pid;
};

#endif
