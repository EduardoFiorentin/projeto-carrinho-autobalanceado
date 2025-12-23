#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double* var_in, double* var_out, double q, double r) {
  Q = q;
  R = r;
  z_n = var_in;
  x_out = var_out; 
  x_est = 0;
  p_est = 2.0;
}

void KalmanFilter::filtre() {
  double p_pred = p_est + Q;
  
  Kn = p_pred / (p_pred + R);          // Kalman gain
  x_est = x_est + Kn * (*z_n - x_est);  // New state value
  p_est = (1 - Kn) * p_pred;           // new range error
  
  *x_out = x_est;
}





