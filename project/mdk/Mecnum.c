#include "zf_common_headfile.h"

#define RADIUS 0.03
#define L 0.0975
float v_fR;
float v_fL;
float v_bR;
float v_bL;
void MecanumControl(float vx, float vy, float vtheta)
{
    float v_fr;
    float v_fl;
    float v_bl;
    float v_br;
    v_fr = (1 / RADIUS) * (vx - vy - L * vtheta);
    v_fl = (1 / RADIUS) * (vx + vy + L * vtheta);
    v_bl = (1 / RADIUS) * (vx + vy - L * vtheta);
    v_br = (1 / RADIUS) * (vx - vy + L * vtheta);
		
}
//解算控制
void motor_set_lf(int pwm )
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR1_DIR, 1);   // 正转
        pwm_set_duty(MOTOR1_PWM, pwm);			
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, 0);   // 反转
        pwm_set_duty(MOTOR1_PWM, -pwm);

    }
}
void motor_set_lb(int pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR3_DIR, 1);  //正转 
        pwm_set_duty(MOTOR3_PWM, pwm);			
    }
    else
    {
        gpio_set_level(MOTOR3_DIR, 0);   
        pwm_set_duty(MOTOR3_PWM, pwm);
    }
}
void motor_set_rb(int pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR3_DIR, 0);   // 正转
        pwm_set_duty(MOTOR3_PWM, pwm);
    }
    else
    {
        gpio_set_level(MOTOR3_DIR, 1);   // 反转
        pwm_set_duty(MOTOR3_PWM, -pwm);
    }
}
void motor_set_rf(int pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR2_DIR, 0);   // 正转
        pwm_set_duty(MOTOR2_PWM, pwm);
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, 1);   // 反转
        pwm_set_duty(MOTOR2_PWM, -pwm);
    }
}
