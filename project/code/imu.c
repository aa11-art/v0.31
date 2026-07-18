#include "zf_common_headfile.h"
#include <math.h>

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} imu_quaternion_t;

static imu_quaternion_t s_quat = {1.0f, 0.0f, 0.0f, 0.0f};

volatile GYRO_t Gyro;

volatile float yaw;

float Gyro_offset_x = 0;
float Gyro_offset_y = 0;
float Gyro_offset_z = 0;

static void imu_quat_normalize(imu_quaternion_t *quat)
{
    float norm = sqrtf(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    if(norm < 1.0e-6f)
    {
        quat->w = 1.0f;
        quat->x = 0.0f;
        quat->y = 0.0f;
        quat->z = 0.0f;
        return;
    }

    quat->w /= norm;
    quat->x /= norm;
    quat->y /= norm;
    quat->z /= norm;
}

static float imu_wrap_angle(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while(angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static void imu_quat_update_x(imu_quaternion_t *quat, float gx_deg_s, float dt)
{
    const float angle_rad = gx_deg_s * dt * (3.1415926f / 180.0f);
    const float half_angle = 0.5f * angle_rad;

    quat->w = cosf(half_angle);
    quat->x = sinf(half_angle);
    quat->y = 0.0f;
    quat->z = 0.0f;

    imu_quat_normalize(quat);
}
////零漂初始化

void offset_init(void)
{
    uint16_t i;

    Gyro_offset_x = 0;
    Gyro_offset_y = 0;
    Gyro_offset_z = 0;

    for (i = 0; i < LOOP_NUM; i++)
    {
        imu660ra_get_gyro();

        Gyro_offset_x += imu660ra_gyro_x;
        Gyro_offset_y += imu660ra_gyro_y;
        Gyro_offset_z += imu660ra_gyro_z;

        system_delay_ms(1);
    }

    Gyro_offset_x /= LOOP_NUM;
    Gyro_offset_y /= LOOP_NUM;
    Gyro_offset_z /= LOOP_NUM;
}

void imu_init(void)
{
    imu660ra_init();

    Kalman_Init(&gyro_kalman);

    offset_init();
}
void get_yaw(void)
{
    float gx_deg_s;
    float gx_filtered;

    /* read gyro and update X-axis angular rate */
    imu660ra_get_gyro();

    /* gyro X (deg/s) with offset removal */
    gx_deg_s = imu660ra_gyro_transition(imu660ra_gyro_x - Gyro_offset_x);

    /* filter gyro reading */
    gx_filtered = Kalman_Filter(&gyro_kalman, gx_deg_s);

    /* integrate gyro X to obtain body correction angle (degrees) */
    yaw += gx_filtered * IMU_DT;
    yaw = imu_wrap_angle(yaw);

    /* expose gyro and keep compatibility */
    Gyro.x = gx_filtered;
    Gyro.y = 0.0f;
    Gyro.z = 0.0f;
}

// end of file