#ifndef PID_H
#define PID_H

#include <cmath>
#include <project2/point.h>
#include <project2/traj.h>

class PID {
 public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, traj prev_goal, traj cur_goal);
 private:
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
};

#endif  /* PID_H */
