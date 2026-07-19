#include "zf_common_headfile.h"
#include "math.h"
#define RADIUS 0.03
#define L 0.35
#define PWM_MAX 4000.0
//PID
float encoder_total;
PID yaw_pid;
volatile float target_yaw = 0.0f;
volatile float target_vx = 0.0f;
volatile float target_vy = 0.0f;
float target_vw = 0.0f;
float target_vtheta = 0.0f;
float now_xx=0.0f;
float now_yy=0.0f;
float current_distance=0.0f;

PID speed_pid[4];
//pid position
PID pos_pid_x;
PID pos_pid_y;
PID pos_pid_w;
PID gyro_w_pid;
//mecanum_PID_init
float now_x=0.0f;
float now_y=0.0f;


void mecanum_pid_init(void)
{
	//角度环
    PID_Init(&yaw_pid, 4.0f, 0.0f, 0.0f, 180.0f, 0.0f);


    // PID_Init(&speed_pid[0],4, 0.2, 0, 4000,4000);
    // PID_Init(&speed_pid[1],4, 0.2, 0, 4000,4000);
    // PID_Init(&speed_pid[2],4, 0.2, 0, 4000,4000);
    // PID_Init(&speed_pid[3],4, 0.2, 0, 4000,4000);
    PID_Init(&speed_pid[0],8.0, 0.3, 0, 4000,4000);
    PID_Init(&speed_pid[1],8.0, 0.3, 0, 4000,4000);
    PID_Init(&speed_pid[2],8.0, 0.3, 0, 4000,4000);
    PID_Init(&speed_pid[3],8.0, 0.3, 0, 4000,4000);
    // PID_Init(&speed_pid[0],5.5, 0.47, 0, 4000,4000);
    // PID_Init(&speed_pid[1],5.5, 0.47, 0, 4000,4000);
    // PID_Init(&speed_pid[2],5.5, 0.47, 0, 4000,4000);
    // PID_Init(&speed_pid[3],5.5, 0.47, 0, 4000,4000);
    // PID_Init(&speed_pid[0],1,0,0,4000,4000);
    // PID_Init(&speed_pid[1],1,0,0,4000,4000);
    // PID_Init(&speed_pid[2],1,0,0,4000,4000);
    // PID_Init(&speed_pid[3],1,0,0,4000,4000);

    // 位置环
    PID_Init(&pos_pid_x,10,0,0,80,0);
    PID_Init(&pos_pid_y,0.5,0,0,50,0);
    // PID_Init(&pos_pid_w,0.5,0,0,200,0);
    // // 角速度环
    PID_Init(&gyro_w_pid, 3, 0.0, 0.0, 500.0, 0.0);
}
//位置环控制函数
// target_x/target_y: 期望沿各轴的目标位移（可为相对距离）
// feedback_x/feedback_y: 当前沿各轴的已行进距离或位置反馈
void PositionControl(float target_x, float target_y, float feedback_x, float feedback_y)
{
    float vx = PID_Calc(&pos_pid_x, target_x, feedback_x);
    float vy = PID_Calc(&pos_pid_y, target_y, feedback_y);

    target_vx = vx;
    target_vy = vy;

}
void x_y_get()
{
     now_xx+=( (float)(encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f)*0.01;
     now_yy+= ((float)(-encoder_fl + encoder_fr + encoder_bl - encoder_br) / 4.0f)*0.01;
}
//位置编码器积分清零
void PositionReset(void)
{
    now_xx = 0.0f;
    now_yy = 0.0f;
}
//角度环函数
float YawControl(float target, float now)
{
    float out;

    /* 处理角度跨越问题 */
    float error = target - now;

    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    out = PID_Calc(&yaw_pid, 0, -error); 

    return out;
}


//Mecnum反向解算vx，vy
void calculate_vx_vy(float v_FL, float v_FR, float v_RL, float v_RR, float *vx, float *vy) {
    // 计算 vx
    *vx = (v_FL + v_FR + v_RL + v_RR) / 4;
    // 计算 vy
    *vy = (-v_FL + v_FR + v_RL - v_RR) / 4;
}


float pwm_fl;
float pwm_fr;
float pwm_bl;
float pwm_br;
// 最内环 四轮速度控制
// 作用 ： 根据目标 前后速度 左右速度 角速度 控制四个轮子达到目标速度
void MecanumMotorSpeedControl(void)
{
    // 将速度转换为PWM（需根据电机额定转速做比例映射，此处简化为直接取整+限幅）
//	encoder_update();
    // v_fL = 80.0;
    // v_fR = 80.0;
    // v_bL = 80.0;
    // v_bR = 80.0;
    pwm_fl = PID_Incremental(&speed_pid[0], v_fL, encoder_fl);
    pwm_fr = PID_Incremental(&speed_pid[1], v_fR, encoder_fr);
    pwm_bl = PID_Incremental(&speed_pid[2], v_bL, encoder_bl);
    pwm_br = PID_Incremental(&speed_pid[3], v_bR, encoder_br);


   // PWM限幅（避免超出硬件范围）
    pwm_fl = fabs(pwm_fl) > PWM_MAX ? (pwm_fl > 0.0 ? PWM_MAX : -PWM_MAX) : pwm_fl;
    pwm_fr = fabs(pwm_fr) > PWM_MAX ? (pwm_fr > 0.0 ? PWM_MAX : -PWM_MAX) : pwm_fr;
    pwm_bl = fabs(pwm_bl) > PWM_MAX ? (pwm_bl > 0.0 ? PWM_MAX : -PWM_MAX) : pwm_bl;
    pwm_br = fabs(pwm_br) > PWM_MAX ? (pwm_br > 0.0 ? PWM_MAX : -PWM_MAX) : pwm_br;

    // 调用电机控制函数执行PWM输出
    motor_set_lf((int32)pwm_fl);
    motor_set_rf((int32)pwm_fr);
    motor_set_lb((int32)pwm_bl);
    motor_set_rb((int32)pwm_br);
    // motor_set_rf(0);
    // motor_set_lb(0);
    // motor_set_rb(0);
}   
float v_fL;
float v_fR;
float v_bL;
float v_bR;
void MecanumCarSpeedControl()
{   
    calculate_vx_vy(encoder_fl,encoder_fr,encoder_bl,encoder_br,&now_x,&now_y);
    float vx = target_vx;
    float vy =  target_vy;
    float target_vw = YawControl(target_yaw, yaw);

    
    float vw = PID_Calc(&gyro_w_pid, target_vw, Gyro.x);
        // float vw = PID_Calc(&gyro_w_pid, 0, Gyro.x);
    // float vw = PID_Calc(&gyro_w_pid, 0, G、yro.x);
    
    // // if(fabs(vw)<50&&vw<0)
    // // {
    // //     vw = -50;
    // // }
    // // else if(fabs(vw)<50&&vw>0)
    // // {
    // //     vw = 50;
    // // }
    // v_fL = target_vx - target_vy - target_vtheta;
    // v_fR = target_vx + target_vy - target_vtheta;
    // v_bL = target_vx + target_vy - target_vtheta;
    // v_bR = target_vx - target_vy + target_vtheta;

    // vw = 0.0;
//    vx=0.0f;
  vy=40.0f;
    // vw=0.0f;
    v_fL = vx - vy - vw;
    v_fR = vx + vy + vw;
    v_bL = vx + vy - vw;
    v_bR = vx - vy + vw;
}

// 使用编码器积分实现前进一定距离的函数
void move_forward_distance(float distance) {
    // 积分距离直到达到目标
    if(current_distance < distance) {
        target_vx = 100.0f; // 设置一个前进速度
        target_vy = 0.0f;  // 不需要侧向移动
        target_vtheta = 0.0f; // 不需要旋转
        float vx = (encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f;
        // 积分距离
        current_distance += vx * 0.01f; // 每10ms调用一次，积分时间为0.01s
    }else{
        target_vx = 0.0f; // 停止前进
        target_vy = 0.0f;  // 不需要侧向移动
        target_vtheta = 0.0f; // 不需要旋转
        current_distance = 0.0f; // 重置距离积分
    }
}
// 使用编码器积分实现前进一定距离的函数
void move_lateral_distance(float distance) {
    // 积分距离直到达到目标
    if(current_distance < distance) {
        target_vx =0.0f; // 设置一个前进速度
        target_vy =100.0f;  // 不需要侧向移动
        target_vtheta = 0.0f; // 不需要旋转
        float vy = (-encoder_fl + encoder_fr + encoder_bl - encoder_br) / 4.0f;
        // 积分距离
        current_distance += vy * 0.01f; // 每10ms调用一次，积分时间为0.01s
    }else{
        target_vx = 0.0f; // 停止前进
        target_vy = 0.0f;  // 不需要侧向移动
        target_vtheta = 0.0f; // 不需要旋转
        // current_distance = 0.0f; // 重置距离积分
    }
}
//解算控制
void motor_set_lf(int32 pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR1_DIR, 1);   // 正转
        pwm_set_duty(MOTOR1_PWM, (uint32)pwm);			
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, 0);   // 反转
        pwm_set_duty(MOTOR1_PWM, (uint32)-pwm);

    }
}
void motor_set_lb(int32 pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR4_DIR, 1);  //正转 
        pwm_set_duty(MOTOR4_PWM, (uint32)pwm);			
    }
    else
    {
        gpio_set_level(MOTOR4_DIR, 0);   
        pwm_set_duty(MOTOR4_PWM, (uint32)-pwm);
    }
}
void motor_set_rb(int32 pwm)
{
    if(pwm >= 0)
    {
        gpio_set_level(MOTOR3_DIR, 0);   // 正转
        pwm_set_duty(MOTOR3_PWM, (uint32)pwm);
    }
    else
    {
        gpio_set_level(MOTOR3_DIR, 1);   // 反转
        pwm_set_duty(MOTOR3_PWM, (uint32)-pwm);
    }
}
void motor_set_rf(int32 pwm)
{
    if(pwm >= 0)
    {
      gpio_set_level(MOTOR2_DIR, 0);   // 正转
        pwm_set_duty(MOTOR2_PWM, (uint32)pwm);
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, 1);   // 反转
        pwm_set_duty(MOTOR2_PWM, (uint32)-pwm);
    }
}
void get_encoder_total(){
	        float v = (encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f;
			encoder_total+=v*0.01;
		
}






void MecanumSpeedPidReset(void)
{
    uint8 i;

    for(i = 0u; i < 4u; i++)
    {
        speed_pid[i].error = 0.0f;
        speed_pid[i].prev_error = 0.0f;
        speed_pid[i].last_error = 0.0f;
        speed_pid[i].output = 0.0f;
    }
}






