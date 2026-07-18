#include "zf_common_headfile.h"

Kalman_t gyro_kalman;

void Kalman_Init(Kalman_t *kalman)
{
    kalman->q = 0.001f;
    kalman->r = 0.1f;
    kalman->x = 0;
    kalman->p = 1;
}

float Kalman_Filter(Kalman_t *kalman, float input)
{
    kalman->p += kalman->q;

    kalman->k = kalman->p / (kalman->p + kalman->r);

    kalman->x += kalman->k * (input - kalman->x);

    kalman->p *= (1 - kalman->k);

    return kalman->x;
}