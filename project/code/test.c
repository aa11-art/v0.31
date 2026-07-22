#include "test.h"
#include "isr.h"
#include "uart.h"
#include "Mecnum.h"
#include "path_executor.h"
#include <math.h>
#include <string.h>

typedef enum
{
    GYRO_SPEED_TUNE_TARGET = 0,
    GYRO_SPEED_TUNE_YAW_KP,
    GYRO_SPEED_TUNE_RATE_KP,
    GYRO_SPEED_TUNE_RATE_KD,
    GYRO_SPEED_TUNE_RUN
} gyro_speed_tune_item_t;

#define GYRO_SPEED_TUNE_GAIN_STEP (0.1f)
#define GYRO_SPEED_TUNE_GAIN_MIN  (0.0f)
#define GYRO_SPEED_TUNE_GAIN_MAX  (20.0f)
#define GYRO_SPEED_TUNE_RATE_DEADBAND  (0.5f)
#define GYRO_SPEED_TUNE_YAW_DEADBAND   (0.5f)
#define GYRO_SPEED_TUNE_RATE_MAX       (50.0f)
#define GYRO_SPEED_TUNE_TARGET_COUNT   (5u)

static const float s_gyro_speed_tune_targets[GYRO_SPEED_TUNE_TARGET_COUNT] =
{
    0.0f, 90.0f, -90.0f, 180.0f, -180.0f
};
static volatile gyro_speed_tune_item_t s_gyro_speed_tune_item = GYRO_SPEED_TUNE_TARGET;
static volatile uint8 s_gyro_speed_tune_running = 0u;
static volatile uint8 s_gyro_speed_tune_target_index = 0u;
static volatile float s_gyro_speed_tune_yaw_kp = 5.0f;
static volatile float s_gyro_speed_tune_kp = 1.5f;
static volatile float s_gyro_speed_tune_kd = 0.0f;
static volatile float s_gyro_speed_tune_yaw_error = 0.0f;
static volatile float s_gyro_speed_tune_rate_target = 0.0f;
static volatile float s_gyro_speed_tune_error = 0.0f;
static volatile float s_gyro_speed_tune_output = 0.0f;
static volatile float s_gyro_speed_tune_peak = 0.0f;
static uint8 s_gyro_speed_tune_wait_release = 0u;
static uint8 s_gyro_speed_tune_screen_tick = 0u;

static uint8 gyro_speed_tune_any_key_pressed(void)
{
    return (uint8)((gpio_get_level(C14) == GPIO_LOW) ||
                   (gpio_get_level(C13) == GPIO_LOW) ||
                   (gpio_get_level(C12) == GPIO_LOW));
}

static float gyro_speed_tune_adjust_gain(float value, int8 direction)
{
    value += (float)direction * GYRO_SPEED_TUNE_GAIN_STEP;
    if(value < GYRO_SPEED_TUNE_GAIN_MIN) value = GYRO_SPEED_TUNE_GAIN_MIN;
    if(value > GYRO_SPEED_TUNE_GAIN_MAX) value = GYRO_SPEED_TUNE_GAIN_MAX;
    if(value < GYRO_SPEED_TUNE_GAIN_STEP * 0.5f) value = 0.0f;
    return value;
}

static float gyro_speed_tune_get_yaw_error(float target, float feedback)
{
    float error = target - feedback;

    if((target == 180.0f) &&
       (fabsf(feedback) <= GYRO_SPEED_TUNE_YAW_DEADBAND))
    {
        return 180.0f - fabsf(feedback);
    }
    if((target == -180.0f) &&
       (fabsf(feedback) <= GYRO_SPEED_TUNE_YAW_DEADBAND))
    {
        return -180.0f + fabsf(feedback);
    }
    if(error > 180.0f) error -= 360.0f;
    if(error < -180.0f) error += 360.0f;
    return error;
}

static void gyro_speed_tune_reset_yaw_pid(void)
{
    yaw_pid.kp = s_gyro_speed_tune_yaw_kp;
    yaw_pid.ki = 0.0f;
    yaw_pid.kd = 0.0f;
    yaw_pid.target = 0.0f;
    yaw_pid.feedback = 0.0f;
    yaw_pid.error = 0.0f;
    yaw_pid.prev_error = 0.0f;
    yaw_pid.last_error = 0.0f;
    yaw_pid.integral = 0.0f;
    yaw_pid.output = 0.0f;
    yaw_pid.max_out = GYRO_SPEED_TUNE_RATE_MAX;
    yaw_pid.max_i = 0.0f;
}

static void gyro_speed_tune_reset_pid(void)
{
    gyro_w_pid.kp = s_gyro_speed_tune_kp;
    gyro_w_pid.ki = 0.0f;
    gyro_w_pid.kd = s_gyro_speed_tune_kd;
    gyro_w_pid.target = 0.0f;
    gyro_w_pid.feedback = 0.0f;
    gyro_w_pid.error = 0.0f;
    gyro_w_pid.prev_error = 0.0f;
    gyro_w_pid.last_error = 0.0f;
    gyro_w_pid.integral = 0.0f;
    gyro_w_pid.output = 0.0f;
    gyro_w_pid.max_out = 500.0f;
    gyro_w_pid.max_i = 0.0f;
}

static void gyro_speed_tune_zero_wheels(void)
{
    v_fL = 0.0f;
    v_fR = 0.0f;
    v_bL = 0.0f;
    v_bR = 0.0f;
}

static void gyro_speed_tune_stop(void)
{
    s_gyro_speed_tune_running = 0u;
    s_gyro_speed_tune_yaw_error = 0.0f;
    s_gyro_speed_tune_rate_target = 0.0f;
    s_gyro_speed_tune_error = 0.0f;
    s_gyro_speed_tune_output = 0.0f;
    gyro_speed_tune_zero_wheels();
    gyro_speed_tune_reset_yaw_pid();
    gyro_speed_tune_reset_pid();
    MecanumSpeedPidReset();
}

void gyro_speed_tune_init(void)
{
    s_gyro_speed_tune_item = GYRO_SPEED_TUNE_TARGET;
    s_gyro_speed_tune_target_index = 0u;
    s_gyro_speed_tune_yaw_kp = 5.0f;
    s_gyro_speed_tune_kp = 1.5f;
    s_gyro_speed_tune_kd = 0.0f;
    s_gyro_speed_tune_peak = 0.0f;
    s_gyro_speed_tune_wait_release = 0u;
    s_gyro_speed_tune_screen_tick = 0u;
    gyro_speed_tune_stop();
    MecanumMotorSpeedControl();
}

void gyro_speed_tune_process_keys(void)
{
    key_scanner();

    if(s_gyro_speed_tune_wait_release != 0u)
    {
        if(gyro_speed_tune_any_key_pressed() == 0u)
        {
            key_clear_all_state();
            s_gyro_speed_tune_wait_release = 0u;
        }
        return;
    }

    if(s_gyro_speed_tune_running != 0u)
    {
        if(gyro_speed_tune_any_key_pressed() != 0u)
        {
            __disable_irq();
            gyro_speed_tune_stop();
            __enable_irq();
            key_clear_all_state();
            s_gyro_speed_tune_wait_release = 1u;
        }
        return;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        s_gyro_speed_tune_item = (gyro_speed_tune_item_t)(
            ((uint8)s_gyro_speed_tune_item + 1u) % 5u);
        key_clear_state(KEY_2);
    }
    else if((key_get_state(KEY_3) == KEY_SHORT_PRESS) ||
            (key_get_state(KEY_4) == KEY_SHORT_PRESS))
    {
        int8 direction = (key_get_state(KEY_3) == KEY_SHORT_PRESS) ? -1 : 1;

        __disable_irq();
        if(s_gyro_speed_tune_item == GYRO_SPEED_TUNE_TARGET)
        {
            if(direction > 0)
            {
                s_gyro_speed_tune_target_index = (uint8)(
                    (s_gyro_speed_tune_target_index + 1u) %
                    GYRO_SPEED_TUNE_TARGET_COUNT);
            }
            else
            {
                s_gyro_speed_tune_target_index = (uint8)(
                    (s_gyro_speed_tune_target_index +
                     GYRO_SPEED_TUNE_TARGET_COUNT - 1u) %
                    GYRO_SPEED_TUNE_TARGET_COUNT);
            }
            gyro_speed_tune_reset_yaw_pid();
            gyro_speed_tune_reset_pid();
            MecanumSpeedPidReset();
        }
        else if(s_gyro_speed_tune_item == GYRO_SPEED_TUNE_YAW_KP)
        {
            s_gyro_speed_tune_yaw_kp = gyro_speed_tune_adjust_gain(
                s_gyro_speed_tune_yaw_kp, direction);
            gyro_speed_tune_reset_yaw_pid();
        }
        else if(s_gyro_speed_tune_item == GYRO_SPEED_TUNE_RATE_KP)
        {
            s_gyro_speed_tune_kp = gyro_speed_tune_adjust_gain(
                s_gyro_speed_tune_kp, direction);
            gyro_speed_tune_reset_pid();
        }
        else if(s_gyro_speed_tune_item == GYRO_SPEED_TUNE_RATE_KD)
        {
            s_gyro_speed_tune_kd = gyro_speed_tune_adjust_gain(
                s_gyro_speed_tune_kd, direction);
            gyro_speed_tune_reset_pid();
        }
        else if((s_gyro_speed_tune_item == GYRO_SPEED_TUNE_RUN) &&
                (direction > 0))
        {
            s_gyro_speed_tune_peak = 0.0f;
            gyro_speed_tune_reset_yaw_pid();
            gyro_speed_tune_reset_pid();
            MecanumSpeedPidReset();
            s_gyro_speed_tune_running = 1u;
        }
        __enable_irq();

        key_clear_all_state();
    }
}

void gyro_speed_tune_update_10ms(void)
{
    float yaw_target;
    float yaw_error;
    float rate_target;
    float rate_error;
    float output;
    float gyro_abs;

    if(s_gyro_speed_tune_running != 0u)
    {
        yaw_target = s_gyro_speed_tune_targets[s_gyro_speed_tune_target_index];
        gyro_abs = fabsf(Gyro.x);
        if(gyro_abs > s_gyro_speed_tune_peak)
        {
            s_gyro_speed_tune_peak = gyro_abs;
        }

        yaw_error = gyro_speed_tune_get_yaw_error(
            yaw_target, yaw);
        if(fabsf(yaw_error) <= GYRO_SPEED_TUNE_YAW_DEADBAND)
        {
            gyro_speed_tune_reset_yaw_pid();
            s_gyro_speed_tune_yaw_error = 0.0f;
            rate_target = 0.0f;
        }
        else
        {
            s_gyro_speed_tune_yaw_error = yaw_error;
            rate_target = YawControl(yaw_target, yaw);
        }
        s_gyro_speed_tune_rate_target = rate_target;

        rate_error = rate_target - Gyro.x;
        if(fabsf(rate_error) <= GYRO_SPEED_TUNE_RATE_DEADBAND)
        {
            gyro_speed_tune_reset_pid();
            s_gyro_speed_tune_error = 0.0f;
            output = 0.0f;
        }
        else
        {
            s_gyro_speed_tune_error = rate_error;
            output = PID_Calc(&gyro_w_pid, rate_target, Gyro.x);
        }
        s_gyro_speed_tune_output = output;

        v_fL = -output;
        v_fR = output;
        v_bL = -output;
        v_bR = output;
    }
    else
    {
        s_gyro_speed_tune_yaw_error = 0.0f;
        s_gyro_speed_tune_rate_target = 0.0f;
        s_gyro_speed_tune_error = 0.0f;
        s_gyro_speed_tune_output = 0.0f;
        gyro_speed_tune_zero_wheels();
    }

    MecanumMotorSpeedControl();
}

void gyro_speed_tune_screen_init(void)
{
    ips200_clear();
    ips200_show_string(0,   0, "YAW CASCADE TUNE");
    ips200_show_string(0,  16, "STATE/ITEM");
    ips200_show_string(0,  32, "YAW KP");
    ips200_show_string(0,  48, "RATE KP");
    ips200_show_string(0,  64, "RATE KD");
    ips200_show_string(0,  80, "TARGET YAW");
    ips200_show_string(0,  96, "YAW");
    ips200_show_string(0, 112, "YAW ERR");
    ips200_show_string(0, 128, "TARGET RATE");
    ips200_show_string(0, 144, "GYRO X");
    ips200_show_string(0, 160, "RATE ERR");
    ips200_show_string(0, 176, "VW OUT");
    ips200_show_string(0, 192, "GYRO PEAK");
    ips200_show_string(0, 208, "K2:NEXT K3:- K4:+");
    ips200_show_string(0, 224, "RUN K4:GO ANY:STOP");
}

void gyro_speed_tune_screen_update(void)
{
    gyro_speed_tune_item_t item;
    uint8 running;
    float yaw_target;
    float yaw_kp;
    float kp;
    float kd;
    float yaw_error;
    float rate_target;
    float gyro_value;
    float error_value;
    float output_value;
    float yaw_value;
    float peak_value;

    if(++s_gyro_speed_tune_screen_tick < 10u)
    {
        return;
    }
    s_gyro_speed_tune_screen_tick = 0u;

    __disable_irq();
    item = s_gyro_speed_tune_item;
    running = s_gyro_speed_tune_running;
    yaw_target = s_gyro_speed_tune_targets[s_gyro_speed_tune_target_index];
    yaw_kp = s_gyro_speed_tune_yaw_kp;
    kp = s_gyro_speed_tune_kp;
    kd = s_gyro_speed_tune_kd;
    yaw_error = s_gyro_speed_tune_yaw_error;
    rate_target = s_gyro_speed_tune_rate_target;
    gyro_value = Gyro.x;
    error_value = s_gyro_speed_tune_error;
    output_value = s_gyro_speed_tune_output;
    yaw_value = yaw;
    peak_value = s_gyro_speed_tune_peak;
    __enable_irq();

    ips200_show_string(96, 16, running ? "RUN " : "STOP");
    switch(item)
    {
        case GYRO_SPEED_TUNE_TARGET:  ips200_show_string(152, 16, "TGT "); break;
        case GYRO_SPEED_TUNE_YAW_KP: ips200_show_string(152, 16, "YKP "); break;
        case GYRO_SPEED_TUNE_RATE_KP: ips200_show_string(152, 16, "RKP "); break;
        case GYRO_SPEED_TUNE_RATE_KD: ips200_show_string(152, 16, "RKD "); break;
        case GYRO_SPEED_TUNE_RUN:     ips200_show_string(152, 16, "RUN "); break;
        default:                      ips200_show_string(152, 16, "ERR "); break;
    }

    ips200_show_float(104,  32, yaw_kp,      5, 1);
    ips200_show_float(104,  48, kp,          5, 1);
    ips200_show_float(104,  64, kd,          5, 1);
    ips200_show_float(104,  80, yaw_target,   7, 1);
    ips200_show_float(104,  96, yaw_value,   7, 1);
    ips200_show_float(104, 112, yaw_error,   7, 1);
    ips200_show_float(104, 128, rate_target, 7, 1);
    ips200_show_float(104, 144, gyro_value,  7, 1);
    ips200_show_float(104, 160, error_value, 7, 1);
    ips200_show_float(104, 176, output_value,7, 1);
    ips200_show_float(104, 192, peak_value,  7, 1);
}

#define POSITION_STEP_TEST_DT_S      (0.01f)
#define POSITION_STEP_TEST_CELL_COUNT (5u)

static sokoban_solution_t s_position_step_solution;
static volatile uint8 s_position_step_direction_index = 0u;
static uint8 s_position_step_wait_release = 0u;
static uint8 s_position_step_screen_tick = 0u;
static volatile uint8 s_position_step_diag_active = 0u;
static volatile float s_position_step_diag_x = 0.0f;
static volatile float s_position_step_diag_y = 0.0f;
static volatile float s_position_step_diag_start_yaw = 0.0f;
static volatile float s_position_step_diag_yaw_delta = 0.0f;
static volatile float s_position_step_diag_yaw_peak = 0.0f;

static void position_step_test_build_route(void)
{
    static const char direction_moves[2] = {'R', 'L'};
    uint16 index;

    (void)memset(&s_position_step_solution, 0,
                 sizeof(s_position_step_solution));
    for(index = 0u; index < POSITION_STEP_TEST_CELL_COUNT; index++)
    {
        s_position_step_solution.move_seq[index] =
            direction_moves[s_position_step_direction_index];
    }
    s_position_step_solution.move_seq[index] = '\0';
    s_position_step_solution.move_count = index;
    s_position_step_solution.solved = 1u;
}

static float position_step_test_normalize_yaw(float angle)
{
    if(angle > 180.0f) angle -= 360.0f;
    if(angle < -180.0f) angle += 360.0f;
    return angle;
}

static void position_step_test_diag_reset(float start_yaw)
{
    __disable_irq();
    s_position_step_diag_active = 1u;
    s_position_step_diag_x = 0.0f;
    s_position_step_diag_y = 0.0f;
    s_position_step_diag_start_yaw = start_yaw;
    s_position_step_diag_yaw_delta = 0.0f;
    s_position_step_diag_yaw_peak = 0.0f;
    __enable_irq();
}

static void position_step_test_diag_update(void)
{
    float measured_vx;
    float measured_vy;
    float yaw_delta;

    if(s_position_step_diag_active == 0u)
    {
        return;
    }

    calculate_vx_vy((float)encoder_fl, (float)encoder_fr,
                    (float)encoder_bl, (float)encoder_br,
                    &measured_vx, &measured_vy);
    s_position_step_diag_x += measured_vx * POSITION_STEP_TEST_DT_S;
    s_position_step_diag_y += measured_vy * POSITION_STEP_TEST_DT_S;

    yaw_delta = position_step_test_normalize_yaw(
        yaw - s_position_step_diag_start_yaw);
    s_position_step_diag_yaw_delta = yaw_delta;
    if(fabsf(yaw_delta) > fabsf(s_position_step_diag_yaw_peak))
    {
        s_position_step_diag_yaw_peak = yaw_delta;
    }
}

static void position_step_test_reset_control(void)
{
    MecanumCarStop();
}

void position_step_test_init(void)
{
    s_position_step_direction_index = 0u;
    s_position_step_wait_release = 0u;
    s_position_step_screen_tick = 0u;
    s_position_step_diag_active = 0u;
    position_step_test_build_route();
    path_executor_abort();
    path_executor_set_heading(SOKOBAN_DIR_UP);
    MecanumSetSpeedDeadzoneMinRatio(
        WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT);
    target_yaw = 0.0f;
    position_step_test_reset_control();
    MecanumMotorSpeedControl();
}

void position_step_test_process_keys(void)
{
    float start_yaw;

    key_scanner();

    if(s_position_step_wait_release != 0u)
    {
        if(gyro_speed_tune_any_key_pressed() == 0u)
        {
            key_clear_all_state();
            s_position_step_wait_release = 0u;
        }
        return;
    }

    if(path_executor_is_idle() == 0u)
    {
        if(gyro_speed_tune_any_key_pressed() != 0u)
        {
            path_executor_abort();
            __disable_irq();
            s_position_step_diag_active = 0u;
            position_step_test_reset_control();
            __enable_irq();
            key_clear_all_state();
            s_position_step_wait_release = 1u;
        }
        return;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        s_position_step_direction_index ^= 1u;
        key_clear_all_state();
    }
    else if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        position_step_test_build_route();
        path_executor_abort();
        position_step_test_reset_control();
        MecanumSetSpeedDeadzoneMinRatio(
            WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT);
        path_executor_set_heading(SOKOBAN_DIR_UP);
        __disable_irq();
        start_yaw = yaw;
        target_yaw = start_yaw;
        __enable_irq();

        if(path_executor_start(&s_position_step_solution) != 0u)
        {
            position_step_test_diag_reset(start_yaw);
        }
        key_clear_all_state();
    }
    else
    {
        key_clear_all_state();
    }
}

void position_step_test_update_10ms(void)
{
    path_executor_update_10ms();
    position_step_test_diag_update();

    if(path_executor_is_idle() == 0u)
    {
        MecanumCarSpeedControl();
    }
    else
    {
        s_position_step_diag_active = 0u;
        position_step_test_reset_control();
    }

    MecanumMotorSpeedControl();
}

void position_step_test_screen_init(void)
{
    ips200_clear();
    ips200_show_string(0,   0, "LATERAL 5 CELL TEST");
    ips200_show_string(0,  16, "DIR");
    ips200_show_string(80, 16, "STATE");
    ips200_show_string(0,  32, "STEP");
    ips200_show_string(0,  48, "POS X/Y");
    ips200_show_string(0,  64, "CARRY");
    ips200_show_string(0,  80, "TGT X/Y");
    ips200_show_string(0,  96, "ERR X/Y");
    ips200_show_string(0, 112, "CMD X/Y");
    ips200_show_string(0, 128, "FL/FR");
    ips200_show_string(0, 144, "BL/BR");
    ips200_show_string(0, 160, "YAW D/PK");
    ips200_show_string(0, 176, "FF/SCL");
    ips200_show_string(0, 208, "K2:DIR");
    ips200_show_string(0, 224, "K4:RUN ANY:STOP");
}

void position_step_test_screen_update(void)
{
    uint8 state;
    uint8 direction_index;
    uint16 step_index;
    uint16 move_count;
    float target_x;
    float target_y;
    float step_position_x;
    float step_position_y;
    float total_position_x;
    float total_position_y;
    float carry_x;
    float carry_y;
    float error_x;
    float error_y;
    float command_vx;
    float command_vy;
    float wheel_fl;
    float wheel_fr;
    float wheel_bl;
    float wheel_br;
    float yaw_delta;
    float yaw_peak;
    float deadzone_min_ratio;
    float odometry_scale;

    if(++s_position_step_screen_tick < 10u)
    {
        return;
    }
    s_position_step_screen_tick = 0u;

    __disable_irq();
    state = path_executor_get_state();
    direction_index = s_position_step_direction_index;
    step_index = path_executor_get_step_index();
    move_count = path_executor_get_move_count();
    target_x = path_executor_get_target_x();
    target_y = path_executor_get_target_y();
    step_position_x = path_executor_get_position_x();
    step_position_y = path_executor_get_position_y();
    total_position_x = s_position_step_diag_x;
    total_position_y = s_position_step_diag_y;
    carry_x = path_executor_get_carry_x();
    carry_y = path_executor_get_carry_y();
    error_x = target_x - step_position_x;
    error_y = target_y - step_position_y;
    command_vx = target_vx;
    command_vy = target_vy;
    wheel_fl = (float)encoder_fl;
    wheel_fr = (float)encoder_fr;
    wheel_bl = (float)encoder_bl;
    wheel_br = (float)encoder_br;
    yaw_delta = s_position_step_diag_yaw_delta;
    yaw_peak = s_position_step_diag_yaw_peak;
    deadzone_min_ratio = MecanumGetSpeedDeadzoneMinRatio();
    __enable_irq();

    odometry_scale = (direction_index == 0u) ?
        1.0f : MECANUM_BODY_LEFT_ODOMETRY_SCALE;

    ips200_show_string(32, 16,
        (direction_index == 0u) ? "RIGHT" : "LEFT ");

    switch(state)
    {
        case 0u: ips200_show_string(128, 16, "IDLE   "); break;
        case 1u: ips200_show_string(128, 16, "LOAD   "); break;
        case 2u: ips200_show_string(128, 16, "RUN    "); break;
        case 3u: ips200_show_string(128, 16, "SETTLE "); break;
        case 4u: ips200_show_string(128, 16, "DONE   "); break;
        case 5u: ips200_show_string(128, 16, "FAULT  "); break;
        default: ips200_show_string(128, 16, "INVALID"); break;
    }

    ips200_show_uint(64, 32, step_index, 3);
    ips200_show_string(96, 32, "/");
    ips200_show_uint(112, 32, move_count, 3);
    ips200_show_float(64,   48, total_position_x,   6, 2);
    ips200_show_string(120, 48, "/");
    ips200_show_float(136,  48, total_position_y,   6, 2);
    ips200_show_float(64,   64, carry_x,            6, 2);
    ips200_show_string(120, 64, "/");
    ips200_show_float(136,  64, carry_y,            6, 2);
    ips200_show_float(64,   80, target_x,           6, 1);
    ips200_show_string(120, 80, "/");
    ips200_show_float(136,  80, target_y,           6, 1);
    ips200_show_float(64,   96, error_x,            6, 2);
    ips200_show_string(120, 96, "/");
    ips200_show_float(136,  96, error_y,            6, 2);
    ips200_show_float(64,  112, command_vx,         6, 1);
    ips200_show_string(120,112, "/");
    ips200_show_float(136, 112, command_vy,         6, 1);
    ips200_show_float(64,  128, wheel_fl,           6, 1);
    ips200_show_string(120,128, "/");
    ips200_show_float(136, 128, wheel_fr,           6, 1);
    ips200_show_float(64,  144, wheel_bl,           6, 1);
    ips200_show_string(120,144, "/");
    ips200_show_float(136, 144, wheel_br,           6, 1);
    ips200_show_float(64,  160, yaw_delta,          6, 1);
    ips200_show_string(120,160, "/");
    ips200_show_float(136, 160, yaw_peak,           6, 1);
    ips200_show_float(64,  176, deadzone_min_ratio, 5, 2);
    ips200_show_string(120,176, "/");
    ips200_show_float(136, 176, odometry_scale,     6, 4);
}

#define LABEL_TEST_OBJECT_TYPE       (1u)
#define LABEL_TEST_OBJECT_INDEX      (0u)
#define LABEL_TEST_OBJECT_ROW        (0u)
#define LABEL_TEST_OBJECT_COL        (0u)
#define LABEL_TEST_BOOT_WAIT_MS      (3000u)
#define LABEL_TEST_REPLY_TIMEOUT_MS  (2000u)
#define LABEL_TEST_POLL_MS           (10u)

static const char g_test_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#--------------#",
    "#----.---.-----#",
    "#----$---$-----#",
    "#----@---------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_single_box_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#--------------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_with_star_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#------*-------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_unsolvable_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#$-------------#",
    "#@------------.#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_labeled_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "#-----$-#-.----#",
    "#------*#------#",
    "#-----@-#------#",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "################"
};

static const char g_two_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "################",
    "################",
    "################",
    "################",
    "#@*#*#$.-------#",
    "################",
    "################",
    "################",
    "################",
    "################",
    "################"
};

static const char g_three_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "################",
    "################",
    "################",
    "################",
    "#@*#*#*#$.-----#",
    "################",
    "################",
    "################",
    "################",
    "################",
    "################"
};

static const char g_optional_bombs_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#-*------------#",
    "#------*-------#",
    "#--------------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_too_many_bombs_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#****----------#",
    "#@$.-----------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static void run_direction_tests(void)
{
    uint8_t heading;
    uint8_t world;
    uint8_t failures = 0u;
    for(heading = 0u; heading < 4u; heading++)
    {
        for(world = 0u; world < 4u; world++)
        {
            uint8_t expected = (uint8_t)((world - heading) & 3u);
            if((uint8_t)sokoban_world_to_body((sokoban_direction_t)heading,
                                              (sokoban_direction_t)world) != expected)
            {
                failures++;
            }
        }
    }
    if((sokoban_direction_turn(SOKOBAN_DIR_UP, 1) != SOKOBAN_DIR_RIGHT) ||
       (sokoban_direction_turn(SOKOBAN_DIR_UP, -1) != SOKOBAN_DIR_LEFT) ||
       (sokoban_direction_turn(SOKOBAN_DIR_UP, 2) != SOKOBAN_DIR_DOWN))
    {
        failures++;
    }
    printf("[direction] failures=%u\r\n", failures);
}

static void run_labeled_bomb_test(void)
{
    sokoban_label_table_t labels;
    sokoban_solution_t solution;
    sokoban_status_t status;
    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 1u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 7u;
    labels.goal_labels[0] = 7u;
    status = sokoban_solve_labeled(g_single_box_map, 3u, 5u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u) || (solution.blast_count != 0u))
    {
        printf("[labeled_no_bomb] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 2u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 1u;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.goal_labels[0] = 1u;
    status = sokoban_solve_labeled(g_labeled_bomb_map, 6u, 6u, &labels, &solution);
    printf("[labeled_bomb] status=%s solved=%u blasts=%u blast=%u,%u move=%u\r\n",
           sokoban_status_string(status), solution.solved, solution.blast_count,
           solution.blast_rows[0], solution.blast_cols[0], solution.move_count);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u) ||
       (solution.blast_count != 1u) ||
       (solution.blast_rows[0] != 5u) || (solution.blast_cols[0] != 8u))
    {
        printf("[labeled_bomb] unexpected result\r\n");
    }
}

static void run_missing_label_tests(void)
{
    sokoban_label_table_t labels;
    sokoban_solution_t solution;
    sokoban_status_t status;

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 1u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 7u;
    status = sokoban_solve_labeled(g_single_box_map, 3u, 5u,
                                   &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
    {
        printf("[missing_goal_label] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 1u;
    labels.goal_count = 1u;
    labels.goal_labels[0] = 7u;
    status = sokoban_solve_labeled(g_single_box_map, 3u, 5u,
                                   &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
    {
        printf("[missing_box_label] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 2u;
    labels.goal_count = 2u;
    labels.box_labels[0] = 3u;
    labels.box_labels[1] = 7u;
    labels.goal_labels[0] = 3u;
    status = sokoban_solve_labeled(g_test_map, 5u, 5u,
                                   &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
    {
        printf("[missing_label_multi_pair] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 2u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 1u;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    status = sokoban_solve_labeled(g_labeled_bomb_map, 6u, 6u,
                                   &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
    {
        printf("[missing_label_with_bomb] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 1u;
    labels.goal_count = 1u;
    status = sokoban_solve_labeled(g_single_box_map, 3u, 5u,
                                   &labels, &solution);
    if(status != SOKOBAN_STATUS_INVALID_LABELS)
    {
        printf("[two_missing_labels] unexpected result\r\n");
    }
}

static void run_multi_bomb_tests(void)
{
    sokoban_label_table_t labels;
    sokoban_solution_t solution;
    sokoban_inspection_plan_t plan;
    sokoban_status_t status;

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 3u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = 7u;
    labels.goal_labels[0] = 7u;
    status = sokoban_solve_labeled(g_two_bomb_map, 5u, 1u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 2u) ||
       (solution.blast_rows[0] != 5u) || (solution.blast_cols[0] != 3u) ||
       (solution.blast_rows[1] != 5u) || (solution.blast_cols[1] != 5u))
    {
        printf("[two_bombs] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 4u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[3] = 9u;
    labels.goal_labels[0] = 9u;
    status = sokoban_solve_labeled(g_three_bomb_map, 5u, 1u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 3u) ||
       (solution.blast_cols[0] != 3u) || (solution.blast_cols[1] != 5u) ||
       (solution.blast_cols[2] != 7u))
    {
        printf("[three_bombs] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 3u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = 4u;
    labels.goal_labels[0] = 4u;
    status = sokoban_solve_labeled(g_optional_bombs_map, 4u, 5u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 0u))
    {
        printf("[optional_bombs] unexpected result\r\n");
    }
    status = sokoban_plan_inspection(g_optional_bombs_map, SOKOBAN_DIR_UP, &plan);
    if((status != SOKOBAN_STATUS_OK) || (plan.event_count != 2u))
    {
        printf("[bomb_inspection] unexpected result\r\n");
    }
    status = sokoban_plan_inspection(g_too_many_bombs_map, SOKOBAN_DIR_UP, &plan);
    if(status != SOKOBAN_STATUS_INVALID_MAP)
    {
        printf("[too_many_bombs] unexpected result\r\n");
    }
}

static void run_inspection_test(void)
{
    sokoban_inspection_plan_t plan;
    sokoban_status_t status = sokoban_plan_inspection(g_test_map, SOKOBAN_DIR_UP, &plan);
    printf("[inspection] status=%s events=%u move=%u final=%u,%u heading=%u\r\n",
           sokoban_status_string(status), plan.event_count, plan.route.move_count,
           plan.final_row, plan.final_col, (uint8_t)plan.final_heading);
    if((status != SOKOBAN_STATUS_OK) || (plan.event_count != 4u) ||
       (plan.final_row >= SOKOBAN_MAP_HEIGHT) || (plan.final_col >= SOKOBAN_MAP_WIDTH))
    {
        printf("[inspection] unexpected result\r\n");
    }
}

static void run_case(const char *name,
                     const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                     uint8_t expect_ok)
{
    sokoban_solution_t solution;
    sokoban_status_t status;

    status = sokoban_solve_decomposed(map, &solution);
    printf("[%s] status=%s solved=%u move=%u push=%u exp_f=%u exp_r=%u\r\n",
           name,
           sokoban_status_string(status),
           solution.solved,
           solution.move_count,
           solution.push_count,
           solution.expanded_forward,
           solution.expanded_reverse);

    if(expect_ok)
    {
        if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
        {
            printf("[%s] unexpected failure\r\n", name);
        }
    }
    else
    {
        if(status == SOKOBAN_STATUS_OK)
        {
            printf("[%s] unexpected success\r\n", name);
        }
    }
}

void sokoban_debug_once(void)
{
    sokoban_solution_t solution;
    sokoban_status_t status;
    char cn_buf[128];

    run_case("single_box_goal", g_single_box_map, 1u);
    run_case("double_box_regression", g_test_map, 1u);
    run_case("bomb_rejected_by_plain_solver", g_with_star_map, 0u);
    run_case("unsolvable", g_unsolvable_map, 0u);
    run_direction_tests();
    run_inspection_test();
    run_labeled_bomb_test();
    run_missing_label_tests();
    run_multi_bomb_tests();

    status = sokoban_solve_decomposed(g_test_map, &solution);

    printf("status=%s\r\n", sokoban_status_string(status));

    if(status == SOKOBAN_STATUS_OK)
    {
        sokoban_moves_to_chinese(solution.move_seq,
                                 solution.move_count,
                                 cn_buf,
                                 sizeof(cn_buf));

        printf("solved=%u\r\n", solution.solved);
        printf("move_count=%u\r\n", solution.move_count);
        printf("push_count=%u\r\n", solution.push_count);
        printf("expanded_f=%u\r\n", solution.expanded_forward);
        printf("expanded_r=%u\r\n", solution.expanded_reverse);
        printf("path_udlr=%s\r\n", solution.move_seq);
        printf("path_cn=%s\r\n", cn_buf);
			ips200_show_uint(0,1,solution.solved,8);
			ips200_show_uint(0,16,solution.move_count,8);
			ips200_show_uint(0,32,solution.push_count,8);
			ips200_show_uint(0,48,solution.expanded_forward,8);
			ips200_show_uint(0,64,solution.expanded_reverse,8);
			ips200_show_string (0,80,solution.move_seq);

    }
}

void label_uart_request_test_once(void)
{
    uint8_t request[8];
    uint8_t rx_ready;
    uint8_t rx_type;
    uint8_t rx_index;
    uint8_t rx_label;
    uint8_t frame_seen;
    uint8_t tail_ok;
    uint8_t check_ok;
    uint8_t match_ok;
    uint8_t comm_ok;
    uint8_t recognition_ok;
    uint32_t waited_ms = 0u;

    request[0] = 0xA5u;
    request[1] = 0xB5u;
    request[2] = LABEL_TEST_OBJECT_TYPE;
    request[3] = LABEL_TEST_OBJECT_INDEX;
    request[4] = LABEL_TEST_OBJECT_ROW;
    request[5] = LABEL_TEST_OBJECT_COL;
    request[6] = (uint8_t)(
        request[2] ^ request[3] ^ request[4] ^ request[5]);
    request[7] = 0xC5u;

    ips200_clear();
    ips200_show_string(0, 0, "UART4 LABEL TEST");
    ips200_show_string(0, 16, "MODE:DIGIT ONLY");
    ips200_show_string(0, 32, "TX:A5 B5 01 00 00 00 01 C5");
    ips200_show_string(0, 48, "BOOT WAIT:3000MS");
    ips200_show_string(0, 64, "VEHICLE TASKS:OFF");
    system_delay_ms(LABEL_TEST_BOOT_WAIT_MS);

    __disable_irq();
    label_rx_ready = 0u;
    label_rx_type = 0u;
    label_rx_index = 0u;
    label_rx_value = 0u;
    label_rx_frame_seen = 0u;
    label_rx_tail_ok = 0u;
    label_rx_check_ok = 0u;
    __enable_irq();

    ips200_show_string(0, 48, "TX STATUS:SENDING");
    label_uart_write_request(request, sizeof(request));
    ips200_show_string(0, 48, "TX STATUS:SENT   ");
    ips200_show_string(0, 80, "RX STATUS:WAITING");

    while(waited_ms < LABEL_TEST_REPLY_TIMEOUT_MS)
    {
        if(label_rx_ready != 0u)
        {
            break;
        }
        system_delay_ms(LABEL_TEST_POLL_MS);
        waited_ms += LABEL_TEST_POLL_MS;
    }

    __disable_irq();
    rx_ready = label_rx_ready;
    rx_type = label_rx_type;
    rx_index = label_rx_index;
    rx_label = label_rx_value;
    frame_seen = label_rx_frame_seen;
    tail_ok = label_rx_tail_ok;
    check_ok = label_rx_check_ok;
    label_rx_ready = 0u;
    __enable_irq();

    match_ok = (uint8_t)(
        (rx_ready != 0u) &&
        (rx_type == LABEL_TEST_OBJECT_TYPE) &&
        (rx_index == LABEL_TEST_OBJECT_INDEX));
    comm_ok = (uint8_t)(
        (rx_ready != 0u) &&
        (tail_ok != 0u) &&
        (check_ok != 0u) &&
        (match_ok != 0u));
    recognition_ok = (uint8_t)(
        (comm_ok != 0u) && (rx_label != 0u));

    ips200_clear();
    ips200_show_string(0, 0, "UART4 LABEL TEST");
    ips200_show_string(0, 16, "TX:A5 B5 01 00 00 00 01 C5");
    ips200_show_string(0, 32, "TX STATUS:SENT");

    ips200_show_string(0, 48, "RX STATUS:");
    if(rx_ready != 0u)
    {
        ips200_show_string(88, 48, "VALID");
    }
    else if(frame_seen != 0u)
    {
        ips200_show_string(88, 48, "INVALID");
    }
    else
    {
        ips200_show_string(88, 48, "TIMEOUT");
    }

    ips200_show_string(0, 64, "RX TYPE:");
    ips200_show_string(0, 80, "RX INDEX:");
    ips200_show_string(0, 96, "RX LABEL:");
    if(rx_ready != 0u)
    {
        ips200_show_uint(88, 64, rx_type, 3);
        ips200_show_uint(88, 80, rx_index, 3);
        ips200_show_uint(88, 96, rx_label, 3);
    }
    else
    {
        ips200_show_string(88, 64, "N/A");
        ips200_show_string(88, 80, "N/A");
        ips200_show_string(88, 96, "N/A");
    }

    ips200_show_string(0, 112, "TAIL:");
    ips200_show_string(88, 112,
        (frame_seen == 0u) ? "N/A" :
        ((tail_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 128, "XOR:");
    ips200_show_string(88, 128,
        (frame_seen == 0u) ? "N/A" :
        ((check_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 144, "MATCH:");
    ips200_show_string(88, 144,
        (rx_ready == 0u) ? "N/A" :
        ((match_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 160, "WAIT MS:");
    ips200_show_uint(88, 160, waited_ms, 4);
    ips200_show_string(0, 176, "COMM:");
    ips200_show_string(88, 176,
        (comm_ok != 0u) ? "PASS" : "FAIL");
    ips200_show_string(0, 192, "RECOG:");
    ips200_show_string(88, 192,
        (recognition_ok != 0u) ? "PASS" : "REJECT");
    ips200_show_string(0, 208, "OVERALL:");
    ips200_show_string(88, 208,
        (recognition_ok != 0u) ? "PASS" : "FAIL");
    ips200_show_string(0, 224, "NO RETRY; TASKS OFF");
}
