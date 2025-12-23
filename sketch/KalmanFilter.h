#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
  private:
    double Q;         // Process noise (model confidence)
    double R;         // 
    double p_init;    // initial confidence 
    double x_init;    // initial state value 

    // I/O variables
    double* z_n;
    double* x_out;

    // state variables 
    double x_est;   // state value 
    double p_est;   // confidence variance
    // double p_pred;      // 
    double Kn; 

  public:
    KalmanFilter(double* var_in, double* var_out, double q, double r);
    void filtre();
};

#endif