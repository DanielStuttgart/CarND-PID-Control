#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Optimize(double tol);

  void Info();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  // add some more variables
  double prev_error;        // previous error for calc. of diff. part
  double int_error;         // sum of error

  std::vector<double> p;
  std::vector<double> dp;

  unsigned int opt_it;      // optimizer iteration
  double best_err;
  int twiddle_step;          // optimize p or dp in this run? 
};

#endif  // PID_H