#ifndef __MECNUM_H
#define __MECNUM_H

#include "zf_common_headfile.h"
#include "PID.h"
#define WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT (0.7f)
//电机                    
#define MOTOR1_DIR               (D3 )   // 左前轮，1 正转
#define MOTOR1_PWM               (PWM2_MODULE3_CHA_D2)

#define MOTOR2_DIR               (C11)   // 右前轮，0 正转
#define MOTOR2_PWM               (PWM2_MODULE2_CHA_C10)


#define MOTOR3_DIR               (C9)    // 右后轮，0 正转
#define MOTOR3_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR4_DIR               (C7)    // 左后轮，1 正转
#define MOTOR4_PWM               (PWM2_MODULE0_CHA_C6)

extern float v_fR;
extern float v_fL;
extern float v_bR;
extern float v_bL;

extern float pwm_fl;
extern float pwm_fr;
extern float pwm_bl;
extern float pwm_br;

extern float now_x;
extern float now_y;

extern float target_x;
extern float target_y;
extern volatile float target_vx;
extern volatile float target_vy;
extern float target_vw;
extern float target_vtheta;
extern volatile float target_yaw;

extern float current_distance;
extern float encoder_total;
extern PID yaw_pid;
extern PID gyro_w_pid;

void MecanumMotorSpeedControl(void);
//位置环控制函数：使用位置 PID 输出目标速度
void PositionControl(float target_x, float target_y, float feedback_x, float feedback_y);
void motor_set_lf(int32 pwm );
void motor_set_rf(int32 pwm );
void motor_set_lb(int32 pwm );
void motor_set_rb(int32 pwm );
void mecanum_pid_init(void);
void MecanumCarSpeedControl(void);
void MecanumCarStop(void);
void calculate_vx_vy(float v_FL, float v_FR, float v_RL, float v_RR, float *vx, float *vy);
float YawControl(float target, float now);
void move_forward_distance(float distance);
void move_lateral_distance(float distance);
void get_encoder_total();
void MecanumSpeedPidReset(void);
void MecanumSetWheelSpeedPidGains(uint8 index, float kp, float ki, float kd);
void MecanumSetSpeedDeadzoneMinRatio(float ratio);
float MecanumGetSpeedDeadzoneMinRatio(void);
#endif
