#include "test.h"
#include "isr.h"
#include "uart.h"
#include "Mecnum.h"
#include "path_executor.h"
#include <math.h>
#include <string.h>

typedef enum
{
    WHEEL_SPEED_TUNE_WHEEL = 0,
    WHEEL_SPEED_TUNE_KP,
    WHEEL_SPEED_TUNE_KI,
    WHEEL_SPEED_TUNE_KD,
    WHEEL_SPEED_TUNE_RUN
} wheel_speed_tune_item_t;

#define WHEEL_SPEED_TUNE_ITEM_COUNT       (5u)
#define WHEEL_SPEED_TUNE_WHEEL_COUNT      (4u)
#define WHEEL_SPEED_TUNE_VX_TARGET        (80.0f)
#define WHEEL_SPEED_TUNE_KP_STEP          (0.1f)
#define WHEEL_SPEED_TUNE_KI_STEP          (0.05f)
#define WHEEL_SPEED_TUNE_KD_STEP          (0.1f)
#define WHEEL_SPEED_TUNE_GAIN_MIN         (0.0f)
#define WHEEL_SPEED_TUNE_KP_MAX           (20.0f)
#define WHEEL_SPEED_TUNE_KI_MAX           (5.0f)
#define WHEEL_SPEED_TUNE_KD_MAX           (20.0f)
#define WHEEL_SPEED_TUNE_TIMEOUT_10MS     (1000u)

static volatile wheel_speed_tune_item_t s_wheel_speed_tune_item =
    WHEEL_SPEED_TUNE_WHEEL;
static volatile uint8 s_wheel_speed_tune_wheel = 0u;
static volatile uint8 s_wheel_speed_tune_running = 0u;
static volatile uint8 s_wheel_speed_tune_timed_out = 0u;
static volatile float s_wheel_speed_tune_kp[WHEEL_SPEED_TUNE_WHEEL_COUNT] =
    {8.0f, 8.0f, 8.0f, 8.0f};
static volatile float s_wheel_speed_tune_ki[WHEEL_SPEED_TUNE_WHEEL_COUNT] =
    {0.3f, 0.3f, 0.3f, 0.3f};
static volatile float s_wheel_speed_tune_kd[WHEEL_SPEED_TUNE_WHEEL_COUNT] =
    {0.0f, 0.0f, 0.0f, 0.0f};
static volatile uint16 s_wheel_speed_tune_run_10ms = 0u;
static uint8 s_wheel_speed_tune_wait_release = 1u;
static uint8 s_wheel_speed_tune_screen_tick = 0u;

static uint8 control_tune_any_key_pressed(void)
{
    return (uint8)((gpio_get_level(C14) == GPIO_LOW) ||
                   (gpio_get_level(C13) == GPIO_LOW) ||
                   (gpio_get_level(C12) == GPIO_LOW));
}

static float wheel_speed_tune_adjust(float value, float step,
                                     float minimum, float maximum,
                                     int8 direction)
{
    value += step * (float)direction;
    if(value < minimum) value = minimum;
    if(value > maximum) value = maximum;
    if((minimum == 0.0f) && (value < step * 0.5f)) value = 0.0f;
    return value;
}

static void wheel_speed_tune_apply_gain(uint8 wheel)
{
    MecanumSetWheelSpeedPidGains(wheel,
                                s_wheel_speed_tune_kp[wheel],
                                s_wheel_speed_tune_ki[wheel],
                                s_wheel_speed_tune_kd[wheel]);
}

static void wheel_speed_tune_stop(uint8 timed_out)
{
    s_wheel_speed_tune_running = 0u;
    s_wheel_speed_tune_timed_out = timed_out;
    v_fL = 0.0f;
    v_fR = 0.0f;
    v_bL = 0.0f;
    v_bR = 0.0f;
    MecanumCarStop();
}

void wheel_speed_tune_init(void)
{
    uint8 wheel;

    s_wheel_speed_tune_item = WHEEL_SPEED_TUNE_WHEEL;
    s_wheel_speed_tune_wheel = 0u;
    for(wheel = 0u; wheel < WHEEL_SPEED_TUNE_WHEEL_COUNT; wheel++)
    {
        s_wheel_speed_tune_kp[wheel] = 10.0f;
        s_wheel_speed_tune_ki[wheel] = 1.0f;
        s_wheel_speed_tune_kd[wheel] = 0.0f;
        wheel_speed_tune_apply_gain(wheel);
    }
    s_wheel_speed_tune_run_10ms = 0u;
    s_wheel_speed_tune_wait_release = 1u;
    s_wheel_speed_tune_screen_tick = 0u;
    wheel_speed_tune_stop(0u);
}

void wheel_speed_tune_process_keys(void)
{
    key_scanner();

    if(s_wheel_speed_tune_running != 0u)
    {
        if(control_tune_any_key_pressed() != 0u)
        {
            __disable_irq();
            wheel_speed_tune_stop(0u);
            __enable_irq();
            key_clear_all_state();
            s_wheel_speed_tune_wait_release = 1u;
        }
        return;
    }

    if(s_wheel_speed_tune_wait_release != 0u)
    {
        if(control_tune_any_key_pressed() == 0u)
        {
            key_clear_all_state();
            s_wheel_speed_tune_wait_release = 0u;
        }
        return;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        s_wheel_speed_tune_item = (wheel_speed_tune_item_t)(
            ((uint8)s_wheel_speed_tune_item + 1u) %
            WHEEL_SPEED_TUNE_ITEM_COUNT);
        key_clear_state(KEY_2);
    }
    else if((key_get_state(KEY_3) == KEY_SHORT_PRESS) ||
            (key_get_state(KEY_4) == KEY_SHORT_PRESS))
    {
        int8 direction =
            (key_get_state(KEY_3) == KEY_SHORT_PRESS) ? -1 : 1;

        __disable_irq();
        s_wheel_speed_tune_timed_out = 0u;
        if(s_wheel_speed_tune_item == WHEEL_SPEED_TUNE_WHEEL)
        {
            s_wheel_speed_tune_wheel = (uint8)(
                (s_wheel_speed_tune_wheel +
                 ((direction > 0) ? 1u :
                  (WHEEL_SPEED_TUNE_WHEEL_COUNT - 1u))) %
                WHEEL_SPEED_TUNE_WHEEL_COUNT);
            MecanumSpeedPidReset();
        }
        else if(s_wheel_speed_tune_item == WHEEL_SPEED_TUNE_KP)
        {
            s_wheel_speed_tune_kp[s_wheel_speed_tune_wheel] =
                wheel_speed_tune_adjust(
                s_wheel_speed_tune_kp[s_wheel_speed_tune_wheel],
                WHEEL_SPEED_TUNE_KP_STEP,
                WHEEL_SPEED_TUNE_GAIN_MIN, WHEEL_SPEED_TUNE_KP_MAX,
                direction);
            wheel_speed_tune_apply_gain(s_wheel_speed_tune_wheel);
        }
        else if(s_wheel_speed_tune_item == WHEEL_SPEED_TUNE_KI)
        {
            s_wheel_speed_tune_ki[s_wheel_speed_tune_wheel] =
                wheel_speed_tune_adjust(
                s_wheel_speed_tune_ki[s_wheel_speed_tune_wheel],
                WHEEL_SPEED_TUNE_KI_STEP,
                WHEEL_SPEED_TUNE_GAIN_MIN, WHEEL_SPEED_TUNE_KI_MAX,
                direction);
            wheel_speed_tune_apply_gain(s_wheel_speed_tune_wheel);
        }
        else if(s_wheel_speed_tune_item == WHEEL_SPEED_TUNE_KD)
        {
            s_wheel_speed_tune_kd[s_wheel_speed_tune_wheel] =
                wheel_speed_tune_adjust(
                s_wheel_speed_tune_kd[s_wheel_speed_tune_wheel],
                WHEEL_SPEED_TUNE_KD_STEP,
                WHEEL_SPEED_TUNE_GAIN_MIN, WHEEL_SPEED_TUNE_KD_MAX,
                direction);
            wheel_speed_tune_apply_gain(s_wheel_speed_tune_wheel);
        }
        else if((s_wheel_speed_tune_item == WHEEL_SPEED_TUNE_RUN) &&
                (direction > 0))
        {
            s_wheel_speed_tune_run_10ms = 0u;
            MecanumSpeedPidReset();
            s_wheel_speed_tune_running = 1u;
        }
        __enable_irq();
        key_clear_all_state();
    }
}

void wheel_speed_tune_update_10ms(void)
{
    if(s_wheel_speed_tune_running != 0u)
    {
        if(s_wheel_speed_tune_run_10ms >=
           WHEEL_SPEED_TUNE_TIMEOUT_10MS)
        {
            wheel_speed_tune_stop(1u);
        }
        else
        {
            s_wheel_speed_tune_run_10ms++;
            v_fL = WHEEL_SPEED_TUNE_VX_TARGET;
            v_fR = WHEEL_SPEED_TUNE_VX_TARGET;
            v_bL = WHEEL_SPEED_TUNE_VX_TARGET;
            v_bR = WHEEL_SPEED_TUNE_VX_TARGET;
            MecanumMotorSpeedControl();
        }
    }
    else
    {
        v_fL = 0.0f;
        v_fR = 0.0f;
        v_bL = 0.0f;
        v_bR = 0.0f;
        MecanumMotorSpeedControl();
    }
}

void wheel_speed_tune_screen_init(void)
{
    ips200_clear();
    ips200_show_string(0,   0, "WHEEL SPEED PID TUNE");
    ips200_show_string(0,  16, "STATE/ITEM");
    ips200_show_string(0,  32, "WHEEL");
    ips200_show_string(0,  48, "VX TARGET");
    ips200_show_string(0,  64, "KP");
    ips200_show_string(0,  80, "KI");
    ips200_show_string(0,  96, "KD");
    ips200_show_string(0, 112, "WHL FB     ERR    PWM");
    ips200_show_string(0, 128, " FL");
    ips200_show_string(0, 144, " FR");
    ips200_show_string(0, 160, " BL");
    ips200_show_string(0, 176, " BR");
    ips200_show_string(0, 192, "RUN SEC");
    ips200_show_string(0, 208, "ALL K4:GO ANY:STOP");
    ips200_show_string(0, 224, "K2:NEXT K3:- K4:+");
}

static void wheel_speed_tune_show_wheel(uint16 y, int feedback,
                                        float target, float output)
{
    ips200_show_int(24, y, feedback, 6);
    ips200_show_int(88, y, (int)(target - (float)feedback), 6);
    ips200_show_int(152, y, (int)output, 6);
}

void wheel_speed_tune_screen_update(void)
{
    wheel_speed_tune_item_t item;
    uint8 running;
    uint8 timed_out;
    uint8 wheel;
    uint16 run_10ms;
    float kp;
    float ki;
    float kd;
    int fl;
    int fr;
    int bl;
    int br;
    float out_fl;
    float out_fr;
    float out_bl;
    float out_br;
    float active_target;

    if(++s_wheel_speed_tune_screen_tick < 10u)
    {
        return;
    }
    s_wheel_speed_tune_screen_tick = 0u;

    __disable_irq();
    item = s_wheel_speed_tune_item;
    running = s_wheel_speed_tune_running;
    timed_out = s_wheel_speed_tune_timed_out;
    wheel = s_wheel_speed_tune_wheel;
    run_10ms = s_wheel_speed_tune_run_10ms;
    kp = s_wheel_speed_tune_kp[wheel];
    ki = s_wheel_speed_tune_ki[wheel];
    kd = s_wheel_speed_tune_kd[wheel];
    fl = encoder_fl;
    fr = encoder_fr;
    bl = encoder_bl;
    br = encoder_br;
    out_fl = pwm_fl;
    out_fr = pwm_fr;
    out_bl = pwm_bl;
    out_br = pwm_br;
    __enable_irq();

    if(running != 0u)
    {
        ips200_show_string(96, 16, "RUN ");
    }
    else if(timed_out != 0u)
    {
        ips200_show_string(96, 16, "TIME");
    }
    else
    {
        ips200_show_string(96, 16, "STOP");
    }

    switch(item)
    {
        case WHEEL_SPEED_TUNE_WHEEL:  ips200_show_string(152, 16, "WHL "); break;
        case WHEEL_SPEED_TUNE_KP:     ips200_show_string(152, 16, "KP  "); break;
        case WHEEL_SPEED_TUNE_KI:     ips200_show_string(152, 16, "KI  "); break;
        case WHEEL_SPEED_TUNE_KD:     ips200_show_string(152, 16, "KD  "); break;
        case WHEEL_SPEED_TUNE_RUN:    ips200_show_string(152, 16, "RUN "); break;
        default:                      ips200_show_string(152, 16, "ERR "); break;
    }

    switch(wheel)
    {
        case 0u: ips200_show_string(96, 32, "FL"); break;
        case 1u: ips200_show_string(96, 32, "FR"); break;
        case 2u: ips200_show_string(96, 32, "BL"); break;
        case 3u: ips200_show_string(96, 32, "BR"); break;
        default: ips200_show_string(96, 32, "??"); break;
    }

    ips200_show_float(96, 48, WHEEL_SPEED_TUNE_VX_TARGET, 7, 1);
    ips200_show_float(96, 64, kp,     6, 1);
    ips200_show_float(96, 80, ki,     6, 2);
    ips200_show_float(96, 96, kd,     6, 1);

    ips200_show_string(0, 128, (wheel == 0u) ? ">FL" : " FL");
    ips200_show_string(0, 144, (wheel == 1u) ? ">FR" : " FR");
    ips200_show_string(0, 160, (wheel == 2u) ? ">BL" : " BL");
    ips200_show_string(0, 176, (wheel == 3u) ? ">BR" : " BR");
    active_target = (running != 0u) ? WHEEL_SPEED_TUNE_VX_TARGET : 0.0f;
    wheel_speed_tune_show_wheel(128, fl,
        active_target, out_fl);
    wheel_speed_tune_show_wheel(144, fr,
        active_target, out_fr);
    wheel_speed_tune_show_wheel(160, bl,
        active_target, out_bl);
    wheel_speed_tune_show_wheel(176, br,
        active_target, out_br);
    ips200_show_float(96, 192, (float)run_10ms * 0.01f, 6, 1);
}

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
    return control_tune_any_key_pressed();
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

#define POSITION_STEP_TEST_DT_S          (0.01f)
#define POSITION_STEP_TEST_CELL_DEFAULT  (5u)
#define POSITION_STEP_TEST_CELL_MAX      (10u)
#define POSITION_STEP_TEST_DIST_DEFAULT  (22.0f)
#define POSITION_STEP_TEST_DIST_STEP     (0.5f)
#define POSITION_STEP_TEST_DIST_MIN      (15.0f)
#define POSITION_STEP_TEST_DIST_MAX      (30.0f)
#define POSITION_STEP_TEST_TOL_DEFAULT   (0.8f)
#define POSITION_STEP_TEST_TOL_STEP      (0.1f)
#define POSITION_STEP_TEST_TOL_MIN       (0.4f)
#define POSITION_STEP_TEST_TOL_MAX       (2.0f)

typedef enum
{
    POSITION_STEP_TEST_ITEM_DIRECTION = 0,
    POSITION_STEP_TEST_ITEM_CELLS,
    POSITION_STEP_TEST_ITEM_DISTANCE,
    POSITION_STEP_TEST_ITEM_TOLERANCE,
    POSITION_STEP_TEST_ITEM_RUN
} position_step_test_item_t;

#define POSITION_STEP_TEST_ITEM_COUNT (5u)

static const char s_position_step_moves[4] = {'U', 'D', 'L', 'R'};
static sokoban_solution_t s_position_step_solution;
static volatile position_step_test_item_t s_position_step_item =
    POSITION_STEP_TEST_ITEM_DIRECTION;
static volatile uint8 s_position_step_direction_index = 3u;
static volatile uint8 s_position_step_cell_count =
    POSITION_STEP_TEST_CELL_DEFAULT;
static volatile float s_position_step_cell_distance =
    POSITION_STEP_TEST_DIST_DEFAULT;
static volatile float s_position_step_tolerance =
    POSITION_STEP_TEST_TOL_DEFAULT;
static volatile uint8 s_position_step_diag_active = 0u;
static volatile uint8 s_position_step_aborted = 0u;
static volatile float s_position_step_total_x = 0.0f;
static volatile float s_position_step_total_y = 0.0f;
static volatile float s_position_step_start_yaw = 0.0f;
static volatile float s_position_step_yaw_delta = 0.0f;
static volatile float s_position_step_yaw_peak = 0.0f;
static uint8 s_position_step_wait_release = 1u;
static uint8 s_position_step_screen_tick = 0u;

static float position_step_test_normalize_yaw(float angle)
{
    if(angle > 180.0f) angle -= 360.0f;
    if(angle < -180.0f) angle += 360.0f;
    return angle;
}

static void position_step_test_clear_measurement(void)
{
    s_position_step_total_x = 0.0f;
    s_position_step_total_y = 0.0f;
    s_position_step_yaw_delta = 0.0f;
    s_position_step_yaw_peak = 0.0f;
}

static float position_step_test_adjust_distance(float value, int8 direction)
{
    value += POSITION_STEP_TEST_DIST_STEP * (float)direction;
    if(value < POSITION_STEP_TEST_DIST_MIN) value = POSITION_STEP_TEST_DIST_MIN;
    if(value > POSITION_STEP_TEST_DIST_MAX) value = POSITION_STEP_TEST_DIST_MAX;
    return value;
}

static float position_step_test_adjust_tolerance(float value, int8 direction)
{
    value += POSITION_STEP_TEST_TOL_STEP * (float)direction;
    if(value < POSITION_STEP_TEST_TOL_MIN) value = POSITION_STEP_TEST_TOL_MIN;
    if(value > POSITION_STEP_TEST_TOL_MAX) value = POSITION_STEP_TEST_TOL_MAX;
    return value;
}

static void position_step_test_build_route(void)
{
    uint16 index;

    (void)memset(&s_position_step_solution, 0,
                 sizeof(s_position_step_solution));
    for(index = 0u; index < s_position_step_cell_count; index++)
    {
        s_position_step_solution.move_seq[index] =
            s_position_step_moves[s_position_step_direction_index];
    }
    s_position_step_solution.move_seq[index] = '\0';
    s_position_step_solution.move_count = index;
    s_position_step_solution.solved = 1u;
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

    measured_vx = (float)(
        encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f;
    measured_vy = (float)(
        -encoder_fl + encoder_fr + encoder_bl - encoder_br) / 4.0f;
    s_position_step_total_x += measured_vx * POSITION_STEP_TEST_DT_S;
    s_position_step_total_y += measured_vy * POSITION_STEP_TEST_DT_S;

    yaw_delta = position_step_test_normalize_yaw(
        yaw - s_position_step_start_yaw);
    s_position_step_yaw_delta = yaw_delta;
    if(fabsf(yaw_delta) > fabsf(s_position_step_yaw_peak))
    {
        s_position_step_yaw_peak = yaw_delta;
    }
}

static void position_step_test_abort_run(void)
{
    path_executor_abort();
    __disable_irq();
    s_position_step_diag_active = 0u;
    s_position_step_aborted = 1u;
    __enable_irq();
    MecanumCarStop();
    s_position_step_wait_release = 1u;
}

static void position_step_test_start(void)
{
    float start_yaw;

    position_step_test_build_route();
    path_executor_abort();
    MecanumCarStop();
    MecanumSetSpeedDeadzoneMinRatio(
        WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT);
    path_executor_set_heading(SOKOBAN_DIR_UP);

    __disable_irq();
    start_yaw = yaw;
    target_yaw = start_yaw;
    s_position_step_start_yaw = start_yaw;
    s_position_step_diag_active = 1u;
    s_position_step_aborted = 0u;
    position_step_test_clear_measurement();
    __enable_irq();

    if(path_executor_start_with_distance_and_tolerance(
           &s_position_step_solution,
           s_position_step_cell_distance,
           s_position_step_tolerance) == 0u)
    {
        __disable_irq();
        s_position_step_diag_active = 0u;
        s_position_step_aborted = 1u;
        __enable_irq();
        MecanumCarStop();
    }
    s_position_step_wait_release = 1u;
}

void position_step_test_init(void)
{
    s_position_step_item = POSITION_STEP_TEST_ITEM_DIRECTION;
    s_position_step_direction_index = 3u;
    s_position_step_cell_count = POSITION_STEP_TEST_CELL_DEFAULT;
    s_position_step_cell_distance = POSITION_STEP_TEST_DIST_DEFAULT;
    s_position_step_tolerance = POSITION_STEP_TEST_TOL_DEFAULT;
    s_position_step_diag_active = 0u;
    s_position_step_aborted = 0u;
    s_position_step_wait_release = 1u;
    s_position_step_screen_tick = 0u;
    position_step_test_clear_measurement();
    path_executor_abort();
    MecanumSetSpeedDeadzoneMinRatio(
        WHEEL_SPEED_LATERAL_DEADZONE_MIN_RATIO_DEFAULT);
    MecanumCarStop();
}

void position_step_test_process_keys(void)
{
    uint8 start_requested = 0u;

    key_scanner();

    if(path_executor_is_idle() == 0u)
    {
        if(gyro_speed_tune_any_key_pressed() != 0u)
        {
            position_step_test_abort_run();
            key_clear_all_state();
        }
        return;
    }

    if(s_position_step_wait_release != 0u)
    {
        if(gyro_speed_tune_any_key_pressed() == 0u)
        {
            key_clear_all_state();
            s_position_step_wait_release = 0u;
        }
        return;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        s_position_step_item = (position_step_test_item_t)(
            ((uint8)s_position_step_item + 1u) %
            POSITION_STEP_TEST_ITEM_COUNT);
    }
    else if((key_get_state(KEY_3) == KEY_SHORT_PRESS) ||
            (key_get_state(KEY_4) == KEY_SHORT_PRESS))
    {
        int8 direction =
            (key_get_state(KEY_3) == KEY_SHORT_PRESS) ? -1 : 1;

        __disable_irq();
        if(s_position_step_item == POSITION_STEP_TEST_ITEM_DIRECTION)
        {
            s_position_step_direction_index = (uint8)(
                (s_position_step_direction_index +
                 ((direction > 0) ? 1u : 3u)) % 4u);
        }
        else if(s_position_step_item == POSITION_STEP_TEST_ITEM_CELLS)
        {
            if((direction < 0) && (s_position_step_cell_count > 1u))
            {
                s_position_step_cell_count--;
            }
            else if((direction > 0) &&
                    (s_position_step_cell_count < POSITION_STEP_TEST_CELL_MAX))
            {
                s_position_step_cell_count++;
            }
        }
        else if(s_position_step_item == POSITION_STEP_TEST_ITEM_DISTANCE)
        {
            s_position_step_cell_distance =
                position_step_test_adjust_distance(
                    s_position_step_cell_distance, direction);
        }
        else if(s_position_step_item == POSITION_STEP_TEST_ITEM_TOLERANCE)
        {
            s_position_step_tolerance =
                position_step_test_adjust_tolerance(
                    s_position_step_tolerance, direction);
        }
        else if((s_position_step_item == POSITION_STEP_TEST_ITEM_RUN) &&
                (direction > 0))
        {
            start_requested = 1u;
        }

        if(start_requested == 0u)
        {
            s_position_step_aborted = 0u;
            position_step_test_clear_measurement();
        }
        __enable_irq();
    }

    key_clear_all_state();
    if(start_requested != 0u)
    {
        position_step_test_start();
    }
}

void position_step_test_update_10ms(void)
{
    path_executor_update_10ms();
    position_step_test_diag_update();

    if(path_executor_is_fault() != 0u)
    {
        s_position_step_diag_active = 0u;
        MecanumCarStop();
    }
    else if(path_executor_is_idle() == 0u)
    {
        MecanumCarSpeedControl();
        MecanumMotorSpeedControl();
    }
    else
    {
        s_position_step_diag_active = 0u;
        MecanumCarStop();
    }
}

void position_step_test_screen_init(void)
{
    ips200_clear();
    ips200_show_string(0,   0, "GRID DISTANCE TUNE");
    ips200_show_string(0,  16, "ITEM");
    ips200_show_string(112,16, "STATE");
    ips200_show_string(0,  32, "DIR");
    ips200_show_string(112,32, "FF");
    ips200_show_string(0,  48, "CELLS");
    ips200_show_string(112,48, "DIST");
    ips200_show_string(0,  64, "TOTAL/TOL");
    ips200_show_string(0,  80, "POS X/Y");
    ips200_show_string(0,  96, "TGT X/Y");
    ips200_show_string(0, 112, "ERR X/Y");
    ips200_show_string(0, 128, "CARRY X/Y");
    ips200_show_string(0, 144, "CMD X/Y");
    ips200_show_string(0, 160, "ENC FL/FR");
    ips200_show_string(0, 176, "ENC BL/BR");
    ips200_show_string(0, 192, "YAW D/PK");
    ips200_show_string(0, 208, "K2:ITEM K3:-");
    ips200_show_string(0, 224, "K4:+/RUN ANY:STOP");
}

void position_step_test_screen_update(void)
{
    position_step_test_item_t item;
    uint8 state;
    uint8 aborted;
    path_executor_fault_reason_t fault_reason;
    uint8 direction_index;
    uint8 cell_count;
    float cell_distance;
    float complete_tolerance;
    float total_target;
    float total_x;
    float total_y;
    float target_x;
    float target_y;
    float position_x;
    float position_y;
    float carry_x;
    float carry_y;
    float command_vx;
    float command_vy;
    float wheel_fl;
    float wheel_fr;
    float wheel_bl;
    float wheel_br;
    float yaw_delta;
    float yaw_peak;
    float deadzone_min_ratio;

    if(++s_position_step_screen_tick < 10u)
    {
        return;
    }
    s_position_step_screen_tick = 0u;

    __disable_irq();
    item = s_position_step_item;
    state = path_executor_get_state();
    aborted = s_position_step_aborted;
    fault_reason = path_executor_get_fault_reason();
    direction_index = s_position_step_direction_index;
    cell_count = s_position_step_cell_count;
    cell_distance = s_position_step_cell_distance;
    complete_tolerance = s_position_step_tolerance;
    total_x = s_position_step_total_x;
    total_y = s_position_step_total_y;
    target_x = path_executor_get_target_x();
    target_y = path_executor_get_target_y();
    position_x = path_executor_get_position_x();
    position_y = path_executor_get_position_y();
    carry_x = path_executor_get_carry_x();
    carry_y = path_executor_get_carry_y();
    command_vx = target_vx;
    command_vy = target_vy;
    wheel_fl = (float)encoder_fl;
    wheel_fr = (float)encoder_fr;
    wheel_bl = (float)encoder_bl;
    wheel_br = (float)encoder_br;
    yaw_delta = s_position_step_yaw_delta;
    yaw_peak = s_position_step_yaw_peak;
    deadzone_min_ratio = MecanumGetSpeedDeadzoneMinRatio();
    __enable_irq();

    total_target = (float)cell_count * cell_distance;

    switch(item)
    {
        case POSITION_STEP_TEST_ITEM_DIRECTION:
            ips200_show_string(40, 16, "DIR "); break;
        case POSITION_STEP_TEST_ITEM_CELLS:
            ips200_show_string(40, 16, "CELL"); break;
        case POSITION_STEP_TEST_ITEM_DISTANCE:
            ips200_show_string(40, 16, "DIST"); break;
        case POSITION_STEP_TEST_ITEM_TOLERANCE:
            ips200_show_string(40, 16, "TOL "); break;
        case POSITION_STEP_TEST_ITEM_RUN:
            ips200_show_string(40, 16, "RUN "); break;
        default:
            ips200_show_string(40, 16, "ERR "); break;
    }

    if(aborted != 0u)
    {
        ips200_show_string(160, 16, "ABORT  ");
    }
    else
    {
        switch(state)
        {
            case 0u: ips200_show_string(160, 16, "IDLE   "); break;
            case 1u: ips200_show_string(160, 16, "LOAD   "); break;
            case 2u: ips200_show_string(160, 16, "RUN    "); break;
            case 3u: ips200_show_string(160, 16, "SETTLE "); break;
            case 4u: ips200_show_string(160, 16, "DONE   "); break;
            case 5u:
                switch(fault_reason)
                {
                    case PATH_EXECUTOR_FAULT_INVALID_MOVE:
                        ips200_show_string(160, 16, "F_CMD  "); break;
                    case PATH_EXECUTOR_FAULT_RUN_TIMEOUT:
                        ips200_show_string(160, 16, "F_RUN  "); break;
                    case PATH_EXECUTOR_FAULT_STOP_TIMEOUT:
                        ips200_show_string(160, 16, "F_STOP "); break;
                    case PATH_EXECUTOR_FAULT_YAW_TIMEOUT:
                        ips200_show_string(160, 16, "F_YAW  "); break;
                    default:
                        ips200_show_string(160, 16, "FAULT  "); break;
                }
                break;
            default: ips200_show_string(160, 16, "INVALID"); break;
        }
    }

    switch(direction_index)
    {
        case 0u: ips200_show_string(32, 32, "FORWARD "); break;
        case 1u: ips200_show_string(32, 32, "BACKWARD"); break;
        case 2u: ips200_show_string(32, 32, "LEFT    "); break;
        case 3u: ips200_show_string(32, 32, "RIGHT   "); break;
        default: ips200_show_string(32, 32, "INVALID "); break;
    }

    ips200_show_float(144, 32, deadzone_min_ratio, 5, 2);
    ips200_show_uint(48, 48, cell_count, 2);
    ips200_show_float(152, 48, cell_distance,       5, 1);
    ips200_show_float(72,  64, total_target,        6, 1);
    ips200_show_string(128, 64, "/");
    ips200_show_float(144, 64, complete_tolerance,  5, 1);
    ips200_show_float(64,  80, total_x,             6, 2);
    ips200_show_string(120, 80, "/");
    ips200_show_float(136, 80, total_y,             6, 2);
    ips200_show_float(64,  96, target_x,            6, 2);
    ips200_show_string(120, 96, "/");
    ips200_show_float(136, 96, target_y,            6, 2);
    ips200_show_float(64, 112, target_x-position_x, 6, 2);
    ips200_show_string(120,112, "/");
    ips200_show_float(136,112, target_y-position_y, 6, 2);
    ips200_show_float(72, 128, carry_x,             6, 2);
    ips200_show_string(128,128, "/");
    ips200_show_float(144,128, carry_y,             6, 2);
    ips200_show_float(64, 144, command_vx,          6, 1);
    ips200_show_string(120,144, "/");
    ips200_show_float(136,144, command_vy,          6, 1);
    ips200_show_float(80, 160, wheel_fl,            5, 1);
    ips200_show_string(128,160, "/");
    ips200_show_float(144,160, wheel_fr,            5, 1);
    ips200_show_float(80, 176, wheel_bl,            5, 1);
    ips200_show_string(128,176, "/");
    ips200_show_float(144,176, wheel_br,            5, 1);
    ips200_show_float(72, 192, yaw_delta,           6, 1);
    ips200_show_string(128,192, "/");
    ips200_show_float(144,192, yaw_peak,            6, 1);
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
