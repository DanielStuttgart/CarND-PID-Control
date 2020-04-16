# CarND-PID-Control
Part of Nanodegree Self-Driving Car Engineer

## Tasks
### Tuning PID parameters
In order to find initial parameters for PID control, I set the throttle to a low value (e.g. 0.1). For detetermining the best parameter values I first adjusted the P-parameter (by setting the other control gains to 0). Then I added the D-control part and last the I-part was added. 
I found following parameters to work quite good: 
* K_p = 0.12
* K_d = 2.8
* K_i = 0.0001

I tried to optimize these parameters with the implemented twiddle algorithm, but could not find parameters better than those which were manually chosen.
If the **proportional** part of the controller is chosen too high, the car starts to swerve and gets unstable. 
The **derivative** term is used for predicting the system behavior and thus improves the stability of the system. 
With the proportional and derivative terms, a permanent error / systematic bias can occur.
The **integral** term is used for compensating this systematic bias. If chosen too high, this integral term may overshoot.
Following animation taken from [Wikipedia](https://en.wikipedia.org/wiki/PID_controller) illustrates the influence of the controller parts.

![](PID_Compensation_Animated.gif)

Following videos show the result with these gain parameters: 

![](/PID_01.gif)

![](/PID_02.gif)

![](/PID_03.gif)

The initialization of the PID controller is shown in following snippet: 
```c++
pid.Init(0.12, 0.0001, 2.8);
```

### Update error based on Cross Track Error (CTE) 
In the following snippet, the error update based on the cross track error is shown: 
```c++
void PID::UpdateError(double cte) {
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  int_error += cte;

  p_error = cte;
  d_error = (cte - prev_error);
  i_error = int_error;

  prev_error = cte;
}
```

### Calculate total error
Since the single error values (P, I, D) have already the correct sign, the total error is simply the sum of all sub-errors:
```c++
return (-Kp * p_error - Kd * d_error - Ki * i_error);
```

### Calculate steering and throttle value in main.cpp
In `main.cpp`, the cross-track-error (CTE) is used to update the errors of the lateral PID-controller. For fine tuning the PID parameters, twiddle-algorithm is executed, if `use_optimizer` is set to true. 
```c++
bool use_optimizer = true; 
pid.UpdateError(cte);         // update error with CTE
if(use_optimizer)
  pid.Optimize(0.001);        // optimize with twiddle

double lat_error = pid.TotalError();    // PID-error is used for steering

// limit the steering value to bin in range [-1, 1]
if(abs(lat_error) <= 1.)
  steer_value = lat_error;
else {
  if (lat_error < -1)
    steer_value = -1;
  else
    steer_value = 1;
}
```
For the longitudinal control, the error is defined as `speed - v_set` with `speed` as current car speed and `v_set` the desired speed (e.g. 35 mph). 

### Twiddle Optimizer
The Twiddle Optimizer is implemented within the function `void PID::Optimize(double tol)`. For porting the twiddle-version shown in the lecture, I implemented a finite state machine which checks the needed part for calculation. Following code snippet shows, how the twiddle optimizer is used:
```c++
main.cpp: pid.UpdateError(cte);
main.cpp: pid.Optimize(0.001);
PID.cpp: p[i] += dp[i]
main.cpp: pid.UpdateError(cte);
main.cpp: pid.Optimize(0.001);
PID.cpp: if (TotalError() < best_err) ? (best_err = TotalError(); dp[i] *= 1.1) : (p[i] -= dp[i]);
main.cpp: pid.UpdateError(cte);
main.cpp: pid.Optimize(0.001);
PID.cpp: if (TotalError() < best_err) ? (best_err = TotalError(); p[i] *= 1.1) : (p[i] += dp[i]; dp[i] *= 0.9);
```
