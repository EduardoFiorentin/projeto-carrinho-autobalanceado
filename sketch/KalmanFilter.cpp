#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double q, double r) {
  // Q - process noise 
  // highter Q  - assumes a volatile "real" state - faster but noiser updates
  // lower Q    - Assumes a stable system - smoother response, but may introduce lag
  Q = q;

  // Measurement noise - describe sthe uncertainty/imprecision of the sensor
  // highter R - tells the filter the sensor is noisy and unreliable - ignore sudden spikes
  R = r;
  x_est = 0;
  p_est = 2.0;        // covariance of extimation error - starts with a arbitrary value and auto-adjusts
  Kn = 0.0;
}

double KalmanFilter::update(double measurement) {
  // prediction step
  // filter tries to predict the uncertainty of the next state 
  // in this model, next state is assumed constant and the uncertainty increases by Q over time
  double p_pred = p_est + Q;
  
  // kalman gain 
  // determines how much to trust the sensor reading vs. the prediction
  Kn = p_pred / (p_pred + R);
  
  // update step 
  // corrects the estimated state based on the actual sensor reading and the kalman gain
  x_est = x_est + Kn * (measurement - x_est);  // New state value
  
  // uncertainty update
  // updates the estimation error covariance for the next iteration
  p_est = (1 - Kn) * p_pred;

  return x_est;
}

double KalmanFilter::value() const {
  return x_est;
}

void KalmanFilter::reset(double initialValue) {
  x_est = initialValue;
  p_est = 2.0;
  Kn = 0.0;
}
