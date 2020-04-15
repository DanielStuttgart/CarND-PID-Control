#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;    

    // initialize internal variables as well
    prev_error = 0;
    int_error = 0;

    // initialize twiddle-var
    p.push_back(Kp);
    p.push_back(Ki);
    p.push_back(Kd);    
    dp.push_back(Kp/5);
    dp.push_back(Ki/5);
    dp.push_back(Kd/5);

    best_err = 0.;
    opt_it = 0;
    twiddle_step = 0;
}

void PID::Optimize(double tol) {    
    // initialize best error after first run
    if (best_err == 0)
        best_err = TotalError();

    if (dp[0] + dp[1] + dp[2] > tol) {
        int i = opt_it % 3;     // choose parameter to optimize
        if (twiddle_step == 0) {            
            p[i] += dp[i];
            twiddle_step = 1;
        }
        else {
            if (twiddle_step == 1) {
                if (TotalError() < best_err) {
                    best_err = TotalError();
                    dp[i] *= 1.1;                    
                } 
                else
                {
                    p[i] -= 2 * dp[i];
                }
                twiddle_step = 2;
            }
            else  // if twiddle_step == 2
            {
                if (TotalError() < best_err) {
                    best_err = TotalError();
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
                twiddle_step = 0;
            }
        }

        // if all 3x parameters were updated
        if(twiddle_step == 0)
            opt_it++;
    }
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */   
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];

    int_error += cte;

    p_error = cte;
    d_error = (cte - prev_error);
    i_error = int_error;

    prev_error = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (-Kp * p_error - Kd * d_error - Ki * i_error);  // TODO: Add your total error calc here!
}

void PID::Info() {
    std::cout << "Kp = " << Kp << "; Ki = " << Ki << "; Kd = " << Kd 
        << "; dp0 = " << dp[0] << "; dp1 = " << dp[1] << "; dp2 = " << dp[2]
        << std::endl;
}