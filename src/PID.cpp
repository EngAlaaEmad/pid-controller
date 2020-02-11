#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  counter = 0;
  // Initialize PID coefficients and errors
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  min_error = 9999999.9;
  max_error = 0.0;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte
  double cte_prev = p_error;
  p_error = cte;
  i_error += cte;
  d_error = cte - cte_prev;

  counter++;

  if (cte > max_error) {
    max_error = cte;
  }
  if (cte < min_error) {
    min_error = cte;
  }
}

double PID::TotalError() {
  // Calculate and return the total error
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::AverageError() { return i_error / counter; }

double PID::MinError() { return min_error; }

double PID::MaxError() { return max_error; }