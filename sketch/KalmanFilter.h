#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
  private:
    double Q;
    double R;

    // state variables 
    double x_est;
    double p_est;
    double Kn;

  public:
    KalmanFilter(double q, double r);
    double update(double measurement);
    double value() const;
    void reset(double initialValue = 0.0);
};

#endif
