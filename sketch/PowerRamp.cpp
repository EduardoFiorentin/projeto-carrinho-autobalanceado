#include "PowerRamp.h"

PowerRamp::PowerRamp(int maxStep, int maxPower)
  : stepLimit(maxStep), powerLimit(maxPower) {
}

int PowerRamp::update(int targetPower) {
  // Clamp the requested target before calculating the rate-limited movement.
  targetPower = constrain(targetPower, 0, powerLimit);

  // Move toward the target by at most stepLimit in either direction.
  int delta = targetPower - currentPower;
  delta = constrain(delta, -stepLimit, stepLimit);
  currentPower += delta;

  return currentPower;
}

int PowerRamp::value() const {
  return currentPower;
}

void PowerRamp::reset() {
  currentPower = 0;
}
