#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double q, double r) {
  // Q - process noise 
  // higher Q - assumes a volatile "real" state - faster but noisier updates
  // lower Q  - assumes a stable system - smoother response, but may introduce lag
  Q = q;

  // Measurement noise - describes the uncertainty/imprecision of the sensor.
  // Higher R tells the filter the sensor is noisy and unreliable.
  R = r;

  // Start with a neutral state and a deliberately loose covariance. The first
  // measurements can then pull the estimate quickly toward reality.
  x_est = 0;
  p_est = 2.0;
  Kn = 0.0;
}

double KalmanFilter::update(double measurement) {
  // Prediction step:
  // The scalar model assumes the next state equals the previous state. Only the
  // uncertainty grows by Q between updates.
  double p_pred = p_est + Q;
  
  // Kalman gain:
  // Determines how much to trust the new measurement versus the prediction.
  Kn = p_pred / (p_pred + R);
  
  // Update step:
  // Correct the estimated state using the innovation (measurement - estimate).
  x_est = x_est + Kn * (measurement - x_est);
  
  // Covariance update:
  // After incorporating the measurement, uncertainty decreases according to Kn.
  p_est = (1 - Kn) * p_pred;

  return x_est;
}

double KalmanFilter::value() const {
  return x_est;
}

void KalmanFilter::reset(double initialValue) {
  // Return to the same uncertainty used at construction while choosing a new
  // starting state.
  x_est = initialValue;
  p_est = 2.0;
  Kn = 0.0;
}
