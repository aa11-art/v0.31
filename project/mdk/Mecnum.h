#ifndef __MECNUM_H
#define __MECNUM_H

//电机                          
#define MOTOR1_DIR               (C11)   //左前轮轮1正转
#define MOTOR1_PWM               (PWM2_MODULE2_CHA_C10)

#define MOTOR2_DIR               (D2 )//右前轮 0正转
#define MOTOR2_PWM               (PWM2_MODULE3_CHB_D3)



#define MOTOR3_DIR               (C6)   //左后轮 1正转
#define MOTOR3_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR4_DIR                (C7) //右后轮 0正转
#define MOTOR4_PWM               (PWM2_MODULE1_CHB_C9)

extern float v_fR;
extern float v_fL;
extern float v_bR;
extern float v_bL;
void MecanumControl(float vx, float vy, float vtheta);
void motor_set_rf(int pwm);
void motor_set_rb(int pwm);
void motor_set_lf(int pwm);
void motor_set_lf(int pwm);


#endif