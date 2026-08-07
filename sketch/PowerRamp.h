#ifndef POWER_RAMP_H
#define POWER_RAMP_H

#include <Arduino.h>

/**
 * Slew-rate limiter for unsigned power magnitudes.
 *
 * The class is preserved for future auxiliary motor behaviors, but it is not
 * used in the main balance loop because balance commands must remain signed
 * and low-latency.
 */
class PowerRamp {
  public:
    PowerRamp(int maxStep, int maxPower);

    // Move currentPower toward targetPower by at most maxStep per call.
    int update(int targetPower);

    // Return the current ramped power.
    int value() const;

    // Drop the ramped value back to zero.
    void reset();

  private:
    int currentPower = 0;
    int stepLimit;
    int powerLimit;
};

#endif
