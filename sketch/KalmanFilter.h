#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/**
 * Scalar Kalman filter kept for signal-smoothing experiments.
 *
 * This is not the main balance angle estimator. It models a single state with a
 * constant-state prediction and one direct measurement input.
 */
class KalmanFilter {
  private:
    // Process and measurement noise parameters.
    double Q;
    double R;

    // Estimated state, covariance, and current Kalman gain.
    double x_est;
    double p_est;
    double Kn;

  public:
    KalmanFilter(double q, double r);

    // Process one scalar measurement and return the new state estimate.
    double update(double measurement);

    // Return the latest state estimate without changing the filter.
    double value() const;

    // Reset covariance and state to a known value.
    void reset(double initialValue = 0.0);
};

#endif
