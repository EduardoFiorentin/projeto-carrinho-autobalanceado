#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float alpha)
  : alpha(alpha) {
}

float ComplementaryFilter::update(float accelAngle, float gyroRate, float dt) {
  // First estimate comes directly from gravity so the filter does not start at
  // a false zero angle.
  if (!hasEstimate) {
    angle = accelAngle;
    hasEstimate = true;
    return angle;
  }

  // Gyro integration predicts fast motion:
  // predictedAngle = previousAngle + angularRate * elapsedTime.
  //
  // The accelerometer term slowly pulls the estimate back to the gravity-based
  // absolute angle, limiting gyro drift.
  angle = alpha * (angle + gyroRate * dt) + (1.0f - alpha) * accelAngle;
  return angle;
}

float ComplementaryFilter::value() const {
  return angle;
}

bool ComplementaryFilter::initialized() const {
  return hasEstimate;
}

void ComplementaryFilter::reset(float initialAngle) {
  // Keep an optional stored value for diagnostics, but force the next update to
  // reinitialize from the current accelerometer angle.
  angle = initialAngle;
  hasEstimate = false;
}
