#include "zf_common_headfile.h"

void PID_Init(PID *pid,float kp,float ki,float kd,float max_out,float max_i)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->max_out = max_out;
    pid->max_i = max_i;

    pid->integral = 0;
    pid->last_error = 0;
}
//position
float PID_Calc(PID *pid,float target,float feedback)
{
    float error = target - feedback;

    pid->integral += error;

    if(pid->integral > pid->max_i) pid->integral = pid->max_i;
    if(pid->integral < -pid->max_i) pid->integral = -pid->max_i;

    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * (error - pid->last_error);

    pid->last_error = error;

    if(output > pid->max_out) output = pid->max_out;
    if(output < -pid->max_out) output = -pid->max_out;

    return output;
}
//increase
float PID_Incremental(PID *pid,float target,float feedback)
{
    float error = target - feedback;

    float delta =
          pid->kp * (error - pid->last_error)
        + pid->ki * error
        + pid->kd * (error - 2*pid->last_error + pid->prev_error);

    pid->output += delta;

    if(pid->output > pid->max_out) pid->output = pid->max_out;
    if(pid->output < -pid->max_out) pid->output = -pid->max_out;

    pid->prev_error = pid->last_error;
    pid->last_error = error;

    return pid->output;
}