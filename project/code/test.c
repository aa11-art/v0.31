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

typedef enum
{
    POSITION_STEP_TUNE_DIRECTION = 0,
    POSITION_STEP_TUNE_DISTANCE,
    POSITION_STEP_TUNE_DEADZONE_MIN_RATIO,
    POSITION_STEP_TUNE_RUN
} position_step_tune_item_t;

#define POSITION_STEP_TUNE_ITEM_COUNT       (4u)
#define POSITION_STEP_TUNE_DISTANCE_DEFAULT (24.0f)
#define POSITION_STEP_TUNE_DISTANCE_STEP    (0.5f)
#define POSITION_STEP_TUNE_DISTANCE_MIN     (0.5f)
#define POSITION_STEP_TUNE_DISTANCE_MAX     (5.0f)
#define POSITION_STEP_TUNE_DEADZONE_RATIO_STEP (0.05f)
#define POSITION_STEP_TUNE_DEADZONE_RATIO_MIN  (0.0f)
#define POSITION_STEP_TUNE_DEADZONE_RATIO_MAX  (1.0f)

static const sokoban_body_direction_t s_position_step_directions[4] =
{
    SOKOBAN_BODY_FORWARD,
    SOKOBAN_BODY_BACKWARD,
    SOKOBAN_BODY_LEFT,
    SOKOBAN_BODY_RIGHT
};
static volatile position_step_tune_item_t s_position_step_item =
    POSITION_STEP_TUNE_DIRECTION;
static volatile uint8 s_position_step_direction_index = 0u;
static volatile float s_position_step_distance =
    POSITION_STEP_TUNE_DISTANCE_DEFAULT;
static uint8 s_position_step_wait_release = 0u;
static uint8 s_position_step_screen_tick = 0u;

static float position_step_test_adjust(float value, float step,
                                       float minimum, float maximum,
                                       int8 direction)
{
    value += step * (float)direction;
    if(value < minimum) value = minimum;
    if(value > maximum) value = maximum;
    return value;
}

static void position_step_test_reset_control(void)
{
    MecanumCarStop();
}

void position_step_test_init(void)
{
    s_position_step_item = POSITION_STEP_TUNE_DIRECTION;
    s_position_step_direction_index = 0u;
    s_position_step_distance = POSITION_STEP_TUNE_DISTANCE_DEFAULT;
    s_position_step_wait_release = 0u;
    s_position_step_screen_tick = 0u;
    path_executor_abort();
    target_yaw = 0.0f;
    position_step_test_reset_control();
    MecanumMotorSpeedControl();
}

void position_step_test_process_keys(void)
{
    uint8 start_requested = 0u;

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
            position_step_test_reset_control();
            __enable_irq();
            key_clear_all_state();
            s_position_step_wait_release = 1u;
        }
        return;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        s_position_step_item = (position_step_tune_item_t)(
            ((uint8)s_position_step_item + 1u) %
            POSITION_STEP_TUNE_ITEM_COUNT);
        key_clear_state(KEY_2);
    }
    else if((key_get_state(KEY_3) == KEY_SHORT_PRESS) ||
            (key_get_state(KEY_4) == KEY_SHORT_PRESS))
    {
        int8 direction = (key_get_state(KEY_3) == KEY_SHORT_PRESS) ? -1 : 1;

        __disable_irq();
        if(s_position_step_item == POSITION_STEP_TUNE_DIRECTION)
        {
            s_position_step_direction_index = (uint8)(
                (s_position_step_direction_index +
                 ((direction > 0) ? 1u : 3u)) % 4u);
        }
        else if(s_position_step_item == POSITION_STEP_TUNE_DISTANCE)
        {
            s_position_step_distance = position_step_test_adjust(
                s_position_step_distance,
                POSITION_STEP_TUNE_DISTANCE_STEP,
                POSITION_STEP_TUNE_DISTANCE_MIN,
                POSITION_STEP_TUNE_DISTANCE_MAX,
                direction);
        }
        else if(s_position_step_item == POSITION_STEP_TUNE_DEADZONE_MIN_RATIO)
        {
            MecanumSetSpeedDeadzoneMinRatio(position_step_test_adjust(
                MecanumGetSpeedDeadzoneMinRatio(),
                POSITION_STEP_TUNE_DEADZONE_RATIO_STEP,
                POSITION_STEP_TUNE_DEADZONE_RATIO_MIN,
                POSITION_STEP_TUNE_DEADZONE_RATIO_MAX,
                direction));
        }
        else if((s_position_step_item == POSITION_STEP_TUNE_RUN) &&
                (direction > 0))
        {
            position_step_test_reset_control();
            start_requested = 1u;
        }
        __enable_irq();

        if(start_requested != 0u)
        {
            path_executor_start_body_step_with_distance(
                s_position_step_directions[s_position_step_direction_index],
                s_position_step_distance);
        }
        key_clear_all_state();
    }
}

void position_step_test_update_10ms(void)
{
    path_executor_update_10ms();

    if(path_executor_is_idle() == 0u)
    {
        MecanumCarSpeedControl();
    }
    else
    {
        position_step_test_reset_control();
    }

    MecanumMotorSpeedControl();
}

void position_step_test_screen_init(void)
{
    ips200_clear();
    ips200_show_string(0,   0, "CELL DISTANCE TEST");
    ips200_show_string(0,  16, "ITEM");
    ips200_show_string(0,  32, "STATE");
    ips200_show_string(0,  48, "DIR");
    ips200_show_string(0,  64, "CELL TARGET");
    ips200_show_string(0,  80, "ENC DIST");
    ips200_show_string(0,  96, "DIST ERR");
    ips200_show_string(0, 112, "ENC X");
    ips200_show_string(0, 128, "ENC Y");
    ips200_show_string(0, 144, "TARGET X/Y");
    ips200_show_string(0, 160, "CMD VX/VY");
    ips200_show_string(0, 176, "YAW");
    ips200_show_string(0, 192, "LAT FF MIN");
    ips200_show_string(0, 208, "K2:ITEM K3:-");
    ips200_show_string(0, 224, "K4:+/RUN ANY:STOP");
}

void position_step_test_screen_update(void)
{
    position_step_tune_item_t item;
    uint8 direction_index;
    uint8 state;
    float cell_distance;
    float encoder_distance;
    float target_x;
    float target_y;
    float position_x;
    float position_y;
    float command_vx;
    float command_vy;
    float yaw_value;
    float deadzone_min_ratio;

    if(++s_position_step_screen_tick < 10u)
    {
        return;
    }
    s_position_step_screen_tick = 0u;

    __disable_irq();
    item = s_position_step_item;
    direction_index = s_position_step_direction_index;
    state = path_executor_get_state();
    cell_distance = s_position_step_distance;
    encoder_distance = path_executor_get_last_step_distance();
    target_x = path_executor_get_target_x();
    target_y = path_executor_get_target_y();
    position_x = path_executor_get_position_x();
    position_y = path_executor_get_position_y();
    command_vx = target_vx;
    command_vy = target_vy;
    yaw_value = yaw;
    deadzone_min_ratio = MecanumGetSpeedDeadzoneMinRatio();
    __enable_irq();

    switch(item)
    {
        case POSITION_STEP_TUNE_DIRECTION: ips200_show_string(104, 16, "DIR "); break;
        case POSITION_STEP_TUNE_DISTANCE:  ips200_show_string(104, 16, "DIST"); break;
        case POSITION_STEP_TUNE_DEADZONE_MIN_RATIO:
            ips200_show_string(104, 16, "FFMN"); break;
        case POSITION_STEP_TUNE_RUN:       ips200_show_string(104, 16, "RUN "); break;
        default:                           ips200_show_string(104, 16, "ERR "); break;
    }

    switch(state)
    {
        case 0u: ips200_show_string(104, 32, "IDLE   "); break;
        case 1u: ips200_show_string(104, 32, "LOAD   "); break;
        case 2u: ips200_show_string(104, 32, "RUN    "); break;
        case 3u: ips200_show_string(104, 32, "SETTLE "); break;
        case 4u: ips200_show_string(104, 32, "DONE   "); break;
        case 5u: ips200_show_string(104, 32, "FAULT  "); break;
        default: ips200_show_string(104, 32, "INVALID"); break;
    }

    switch(s_position_step_directions[direction_index])
    {
        case SOKOBAN_BODY_FORWARD:  ips200_show_string(104, 48, "FORWARD "); break;
        case SOKOBAN_BODY_BACKWARD: ips200_show_string(104, 48, "BACKWARD"); break;
        case SOKOBAN_BODY_LEFT:     ips200_show_string(104, 48, "LEFT    "); break;
        case SOKOBAN_BODY_RIGHT:    ips200_show_string(104, 48, "RIGHT   "); break;
        default:                    ips200_show_string(104, 48, "INVALID "); break;
    }

    ips200_show_float(104,  64, cell_distance,                    7, 1);
    ips200_show_float(104,  80, encoder_distance,                 7, 2);
    ips200_show_float(104,  96, cell_distance-encoder_distance,  7, 2);
    ips200_show_float(104, 112, position_x,                       7, 2);
    ips200_show_float(104, 128, position_y,                       7, 2);
    ips200_show_float(64,  144, target_x,                         6, 1);
    ips200_show_string(120, 144, "/");
    ips200_show_float(136, 144, target_y,                         6, 1);
    ips200_show_float(64,  160, command_vx,                       6, 1);
    ips200_show_string(120, 160, "/");
    ips200_show_float(136, 160, command_vy,                       6, 1);
    ips200_show_float(104, 176, yaw_value,                        7, 1);
    ips200_show_float(104, 192, deadzone_min_ratio,               7, 2);
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
