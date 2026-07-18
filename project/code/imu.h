#ifndef __IMU_H
#define __IMU_H

#include "zf_common_headfile.h"
#define IMU_DT   0.001   // 1ms
#define LOOP_NUM 1000

typedef struct
{
    float x;
    float y;
    float z;

}GYRO_t;

extern volatile GYRO_t Gyro;

void get_yaw(void);
void offset_init();
void imu_init(void);

extern volatile float yaw;
extern float Gyro_offset_z;

#endif