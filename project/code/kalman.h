#ifndef __KALMAN_H
#define __KALMAN_H

#include "zf_common_headfile.h"

typedef struct
{
    float q;
    float r;
    float x;
    float p;
    float k;

}Kalman_t;

extern Kalman_t gyro_kalman;

void Kalman_Init(Kalman_t *kalman);
float Kalman_Filter(Kalman_t *kalman, float input);

#endif