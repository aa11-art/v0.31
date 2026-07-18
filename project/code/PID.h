#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;
    float feedback;

    float error;
    float prev_error;
    float last_error;

    float integral;
    float output;

    float max_out;
    float max_i;

}PID;

void PID_Init(PID *pid,float kp,float ki,float kd,float max_out,float max_i);
float PID_Calc(PID *pid,float target,float feedback);
float PID_Incremental(PID *pid,float target,float feedback);


#endif