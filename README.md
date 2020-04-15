# CarND-PID-Control
Part of Nanodegree Self-Driving Car Engineer

## Project overview

## Tasks
### Initialize PID controller
Initialize PID controller with values from the lessons.
```c++
pid.Init(0.2, 0.004, 3.0);
```
Following part is implemented in PID.cpp:
```c++
Kp = Kp_;
Kd = Kd_;
Ki = Ki_;

// initialize internal variables as well
prev_error = 0;
int_error = 0;
```

### Update error based on Cross Track Error (CTE) 
```c++
int_error += cte;
p_error = -Kp * cte;
d_error = -Kd * (cte - prev_error);
i_error = -Ki * int_error;

prev_error = cte;
```

### Calculate total error
Since the single error values (P, I, D) have already the correct sign, the total error is simply the sum of all sub-errors:
```c++
return p_error + d_error + i_error;
```

### Calculate steering value in main.cpp
```c++

```

### Initialize Parameters
In order to find initial parameters for PID control, I set the throttle to a low value (e.g. 0.1). For detetermining the best parameter values I first adjusted the P-parameter (by setting the other control gains to 0). Then I added the D-control part and last the I-part was added. 
I found following parameters to work quite good: 
* K_p = 0.075
* K_d = 2.75
* K_i = 0.0001
