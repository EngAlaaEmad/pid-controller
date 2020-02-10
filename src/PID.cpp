#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID coefficients and errors
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte
  double cte_prev = p_error;
  p_error = Kp * cte;
  i_error += Ki * cte;
  d_error = Kd * (cte - cte_prev);
}

double PID::TotalError() {
  // Calculate and return the total error
  return p_error + i_error + d_error;
}