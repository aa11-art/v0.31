#include "zf_common_headfile.h"
#include "math.h"
#define RADIUS 0.03
#define L 0.35
#define PWM_MAX 4000.0
#define WHEEL_SPEED_TARGET_EPSILON 0.01f
#define WHEEL_SPEED_DEADZONE_FULL_TARGET 30.0f
#define YAW_ERROR_DEADBAND 0.5f
#define GYRO_RATE_ERROR_DEADBAND 0.5f
//PID
float encoder_total;
PID yaw_pid;
volatile float target_yaw = 0.0f;
volatile float target_vx = 0.0f;
volatile float target_vy = 0.0f;
float target_vw = 0.0f;
float target_vtheta = 0.0f;
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
    PID_Init(&yaw_pid, 5.0f, 0.0f, 0.0f, 400.0f, 0.0f);


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
    PID_Init(&pos_pid_x,7,0,0,55,0);
    PID_Init(&pos_pid_y,8.5,0,0,80,0);
    // PID_Init(&pos_pid_w,0.5,0,0,200,0);
    // // 角速度环
    PID_Init(&gyro_w_pid, 1.5f, 0.0f, 0.0f, 500.0f, 0.0f);
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
/* Logical wheel order: FL/M1, FR/M2, BL/M4, BR/M3. */
static const float s_speed_deadzone_positive[4] = {625.0f, 900.0f, 600.0f, 675.0f};
static const float s_speed_deadzone_negative[4] = {600.0f, 975.0f, 875.0f, 850.0f};
static int8 s_speed_target_direction[4] = {0, 0, 0, 0};
static volatile float s_speed_deadzone_min_ratio =
    WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT;

void MecanumSetSpeedDeadzoneMinRatio(float ratio)
{
    if(ratio < 0.0f) ratio = 0.0f;
    if(ratio > 1.0f) ratio = 1.0f;
    s_speed_deadzone_min_ratio = ratio;
}

float MecanumGetSpeedDeadzoneMinRatio(void)
{
    return s_speed_deadzone_min_ratio;
}

static void MecanumSpeedPidResetOne(uint8 index)
{
    speed_pid[index].target = 0.0f;
    speed_pid[index].feedback = 0.0f;
    speed_pid[index].error = 0.0f;
    speed_pid[index].prev_error = 0.0f;
    speed_pid[index].last_error = 0.0f;
    speed_pid[index].integral = 0.0f;
    speed_pid[index].output = 0.0f;
    s_speed_target_direction[index] = 0;
}

static float MecanumWheelSpeedControl(uint8 index, float target, float feedback,
                                      float deadzone_min_ratio)
{
    int8 direction;
    float feedforward_ratio;
    float output;

    if(fabsf(target) < WHEEL_SPEED_TARGET_EPSILON)
    {
        MecanumSpeedPidResetOne(index);
        return 0.0f;
    }

    direction = (target > 0.0f) ? 1 : -1;
    if((s_speed_target_direction[index] != 0) &&
       (s_speed_target_direction[index] != direction))
    {
        MecanumSpeedPidResetOne(index);
    }
    s_speed_target_direction[index] = direction;

    output = PID_Incremental(&speed_pid[index], target, feedback);
    feedforward_ratio = fabsf(target) / WHEEL_SPEED_DEADZONE_FULL_TARGET;
    if(feedforward_ratio < deadzone_min_ratio)
    {
        feedforward_ratio = deadzone_min_ratio;
    }
    if(feedforward_ratio > 1.0f)
    {
        feedforward_ratio = 1.0f;
    }
    output += ((direction > 0) ? s_speed_deadzone_positive[index] :
                                -s_speed_deadzone_negative[index]) * feedforward_ratio;

    if(output > PWM_MAX)
    {
        output = PWM_MAX;
    }
    else if(output < -PWM_MAX)
    {
        output = -PWM_MAX;
    }

    return output;
}
// 最内环 四轮速度控制
// 作用 ： 根据目标 前后速度 左右速度 角速度 控制四个轮子达到目标速度
void MecanumMotorSpeedControl(void)
{
    float deadzone_min_ratio = 0.0f;

    if((fabsf(target_vy) > WHEEL_SPEED_TARGET_EPSILON) &&
       (fabsf(target_vy) > fabsf(target_vx)))
    {
        deadzone_min_ratio = s_speed_deadzone_min_ratio;
    }

    // 将速度转换为PWM（需根据电机额定转速做比例映射，此处简化为直接取整+限幅）
//	encoder_update();
    // v_fL = 80.0;
    // v_fR = 80.0;
    // v_bL = 80.0;
    // v_bR = 80.0;
    pwm_fl = MecanumWheelSpeedControl(0u, v_fL, encoder_fl, deadzone_min_ratio);
    pwm_fr = MecanumWheelSpeedControl(1u, v_fR, encoder_fr, deadzone_min_ratio);
    pwm_bl = MecanumWheelSpeedControl(2u, v_bL, encoder_bl, deadzone_min_ratio);
    pwm_br = MecanumWheelSpeedControl(3u, v_bR, encoder_br, deadzone_min_ratio);
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
void MecanumCarSpeedControl(void)
{
    float yaw_error;
    float rate_error;
    float vx;
    float vy;
    float vw;

    calculate_vx_vy(encoder_fl,encoder_fr,encoder_bl,encoder_br,&now_x,&now_y);
    vx = target_vx;
    vy = target_vy;

    yaw_error = target_yaw - yaw;
    if(yaw_error > 180.0f) yaw_error -= 360.0f;
    if(yaw_error < -180.0f) yaw_error += 360.0f;
    if(fabsf(yaw_error) <= YAW_ERROR_DEADBAND)
    {
        target_vw = 0.0f;
    }
    else
    {
        target_vw = YawControl(target_yaw, yaw);
    }

    rate_error = target_vw - Gyro.x;
    if(fabsf(rate_error) <= GYRO_RATE_ERROR_DEADBAND)
    {
        vw = 0.0f;
    }
    else
    {
        vw = PID_Calc(&gyro_w_pid, target_vw, Gyro.x);
    }
    target_vtheta = vw;

    v_fL = vx - vy - vw;
    v_fR = vx + vy + vw;
    v_bL = vx + vy - vw;
    v_bR = vx - vy + vw;
}

void MecanumCarStop(void)
{
    target_vx = 0.0f;
    target_vy = 0.0f;
    target_vw = 0.0f;
    target_vtheta = 0.0f;
    v_fL = 0.0f;
    v_fR = 0.0f;
    v_bL = 0.0f;
    v_bR = 0.0f;
    yaw_pid.integral = 0.0f;
    yaw_pid.last_error = 0.0f;
    gyro_w_pid.integral = 0.0f;
    gyro_w_pid.last_error = 0.0f;
    MecanumSpeedPidReset();
    pwm_fl = 0.0f;
    pwm_fr = 0.0f;
    pwm_bl = 0.0f;
    pwm_br = 0.0f;
    motor_set_lf(0);
    motor_set_rf(0);
    motor_set_lb(0);
    motor_set_rb(0);
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
        MecanumSpeedPidResetOne(i);
    }
}






