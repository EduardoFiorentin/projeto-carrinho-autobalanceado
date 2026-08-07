#ifndef POWER_RAMP_H
#define POWER_RAMP_H

#include <Arduino.h>

class PowerRamp {
  public:
    PowerRamp(int maxStep, int maxPower);

    int update(int targetPower);
    int value() const;
    void reset();

  private:
    int currentPower = 0;
    int stepLimit;
    int powerLimit;
};

#endif
