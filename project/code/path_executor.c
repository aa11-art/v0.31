#include "zf_common_headfile.h"
#include <math.h>
#include "path_executor.h"
#include "Mecnum.h"

// 路径执行器相关宏定义
#define PATH_EXECUTOR_DT_S            (0.01f)     //时间步长（秒）
#define PATH_EXECUTOR_CELL_DISTANCE  (24.0f)      // 每个单元格的距离
#define PATH_EXECUTOR_COMPLETE_TOL   (1.2f)       // 完成容差
#define PATH_EXECUTOR_CROSS_AXIS_TOL (0.5f)       // 非运动轴完成容差
#define PATH_EXECUTOR_COMPLETE_TICKS (8.0f)         // 连续到位 ticks 数
#define PATH_EXECUTOR_SETTLE_TICKS   (5u)         // 稳定 ticks 数
#define PATH_EXECUTOR_SETTLE_SPEED   (2.0f)       // 车体停稳速度阈值
#define PATH_EXECUTOR_SETTLE_YAW     (2.0f)       // 停稳航向误差
#define PATH_EXECUTOR_SETTLE_RATE    (3.0f)       // 停稳角速度
#define PATH_EXECUTOR_SETTLE_TIMEOUT (100u)       // 停稳最长等待（1000 ms）
#define PATH_EXECUTOR_TIMEOUT_TICKS  (500u)       // 单步超时（5 秒）



// 路径执行器状态枚举
typedef enum
{
    PATH_EXECUTOR_IDLE = 0,     // 空闲状态
    PATH_EXECUTOR_LOAD_STEP,    // 加载步骤
    PATH_EXECUTOR_RUN_STEP,     // 执行步骤
    PATH_EXECUTOR_SETTLE,       // 稳定状态
    PATH_EXECUTOR_DONE,         // 完成
    PATH_EXECUTOR_FAULT         // 故障
} path_executor_state_t;

// 路径步骤轴枚举
typedef enum
{
    PATH_AXIS_VX = 0,           // X轴
    PATH_AXIS_VY                // Y轴
} path_step_axis_t;

static float s_carry_error_x = 0.0f;
static float s_carry_error_y = 0.0f;
static volatile uint8_t s_settle_elapsed_ticks = 0u;
static volatile path_executor_fault_reason_t s_fault_reason =
    PATH_EXECUTOR_FAULT_NONE;

// 静态变量声明
static volatile path_executor_state_t s_state = PATH_EXECUTOR_IDLE;  // 当前状态
static char s_move_seq[SOKOBAN_MAX_MOVES + 1u];                      // 移动序列
static uint16_t s_move_count = 0u;                                   // 移动计数
static volatile uint16_t s_step_index = 0u;                          // 当前步骤索引
static volatile uint16_t s_step_cell_count = 1u;                     // 当前连续同方向格数
static float s_step_position_x = 0.0f;                               // 单步X位置
static float s_step_position_y = 0.0f;                               // 单步Y位置
static float s_step_target_x = 0.0f;                                 // 单步X目标
static float s_step_target_y = 0.0f;                                 // 单步Y目标
static float s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;          // 步骤距离
static float s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL; // 运动轴完成容差
static volatile uint32_t s_step_elapsed_ticks = 0u;                  // 连续段运行 ticks 数
static volatile uint8_t s_complete_ticks = 0u;                       // 连续到位 ticks 数
static volatile uint8_t s_settle_ticks = 0u;                         // 稳定ticks计数
static path_step_axis_t s_step_axis = PATH_AXIS_VX;                  // 当前轴
static volatile sokoban_direction_t s_heading = SOKOBAN_DIR_RIGHT;

// 停止运动
// 停止运动
static void path_executor_stop_motion(void)
{
    target_vx = 0.0f;
    target_vy = 0.0f;
}

static void path_executor_enter_fault(path_executor_fault_reason_t reason)
{
    s_carry_error_x = 0.0f;
    s_carry_error_y = 0.0f;
    s_step_cell_count = 1u;
    s_fault_reason = reason;
    path_executor_stop_motion();
    MecanumSpeedPidReset();
    s_state = PATH_EXECUTOR_FAULT;
}

// 测量X轴速度
static float path_executor_measure_vx(void)
{
    return (float)(encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f;
}

// 测量Y轴速度
static float path_executor_measure_vy(void)
{
    return
        (float)(-encoder_fl + encoder_fr + encoder_bl - encoder_br) / 4.0f;
}

static float path_executor_get_yaw_error(void)
{
    float error = target_yaw - yaw;

    if(error > 180.0f) error -= 360.0f;
    if(error < -180.0f) error += 360.0f;
    return error;
}

static uint8_t path_executor_body_settled(float measured_vx,
                                           float measured_vy,
                                           float yaw_error)
{
    return (uint8_t)((fabsf(measured_vx) <= PATH_EXECUTOR_SETTLE_SPEED) &&
                     (fabsf(measured_vy) <= PATH_EXECUTOR_SETTLE_SPEED) &&
                     (fabsf(yaw_error) <= PATH_EXECUTOR_SETTLE_YAW) &&
                     (fabsf(Gyro.x) <= PATH_EXECUTOR_SETTLE_RATE));
}

static uint8_t path_executor_within_tolerance(float error_x,
                                               float error_y,
                                               uint8_t *within_x,
                                               uint8_t *within_y)
{
    if(s_step_axis == PATH_AXIS_VX)
    {
        *within_x = (uint8_t)(fabsf(error_x) <= s_step_complete_tolerance);
        *within_y = (uint8_t)(fabsf(error_y) <= PATH_EXECUTOR_CROSS_AXIS_TOL);
    }
    else
    {
        *within_x = (uint8_t)(fabsf(error_x) <= PATH_EXECUTOR_CROSS_AXIS_TOL);
        *within_y = (uint8_t)(fabsf(error_y) <= s_step_complete_tolerance);
    }

    return (uint8_t)(*within_x && *within_y);
}

static uint8_t path_executor_prepare_body_step(sokoban_body_direction_t body_direction)
{
    float run_distance = s_step_distance * (float)s_step_cell_count;

    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = s_carry_error_x;
    s_step_target_y = s_carry_error_y;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
    s_settle_elapsed_ticks = 0u;

    switch(body_direction)
    {
        case SOKOBAN_BODY_FORWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_x += run_distance; break;
        case SOKOBAN_BODY_BACKWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_x -= run_distance; break;
        case SOKOBAN_BODY_LEFT:
            s_step_axis = PATH_AXIS_VY; s_step_target_y += run_distance; break;
        case SOKOBAN_BODY_RIGHT:
            s_step_axis = PATH_AXIS_VY; s_step_target_y -= run_distance; break;
        default: return 0u;
    }
    return 1u;
}

// 准备世界坐标路径步骤
static uint8_t path_executor_prepare_step(char move)
{
    sokoban_direction_t world_direction;

    switch(move)
    {
        case 'U': world_direction = SOKOBAN_DIR_UP; break;
        case 'D': world_direction = SOKOBAN_DIR_DOWN; break;
        case 'L': world_direction = SOKOBAN_DIR_LEFT; break;
        case 'R': world_direction = SOKOBAN_DIR_RIGHT; break;
        default: return 0u;
    }
    return path_executor_prepare_body_step(
        sokoban_world_to_body(s_heading, world_direction));
}
 
// 初始化路径执行器
void path_executor_init(void)
{
    uint16_t idx;

    for(idx = 0u; idx <= SOKOBAN_MAX_MOVES; idx++)
    {
        s_move_seq[idx] = '\0';
    }

    s_move_count = 0u;
    s_step_index = 0u;
    s_step_cell_count = 1u;
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_carry_error_x = 0.0f;
    s_carry_error_y = 0.0f;
    s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
    s_settle_elapsed_ticks = 0u;
    s_fault_reason = PATH_EXECUTOR_FAULT_NONE;
    s_step_axis = PATH_AXIS_VX;
    s_state = PATH_EXECUTOR_IDLE;
    s_heading = SOKOBAN_DIR_UP;
    target_yaw = 0.0f;
    path_executor_stop_motion();
}

void path_executor_abort(void)
{
    __disable_irq();
    s_state = PATH_EXECUTOR_IDLE;
    s_move_count = 0u;
    s_step_index = 0u;
    s_step_cell_count = 1u;
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_carry_error_x = 0.0f;
    s_carry_error_y = 0.0f;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
    s_settle_elapsed_ticks = 0u;
    s_fault_reason = PATH_EXECUTOR_FAULT_NONE;
    path_executor_stop_motion();
    MecanumSpeedPidReset();
    __enable_irq();
}

uint8_t path_executor_start_body_step(sokoban_body_direction_t direction)
{
    return path_executor_start_body_step_with_distance(
        direction, PATH_EXECUTOR_CELL_DISTANCE);
}

uint8_t path_executor_start_body_step_with_distance(
    sokoban_body_direction_t direction, float step_distance)
{
    return path_executor_start_body_step_with_distance_and_tolerance(
        direction, step_distance, PATH_EXECUTOR_COMPLETE_TOL);
}

uint8_t path_executor_start_body_step_with_distance_and_tolerance(
    sokoban_body_direction_t direction,
    float step_distance,
    float complete_tolerance)
{
    if(((uint8_t)direction > (uint8_t)SOKOBAN_BODY_LEFT) ||
       (step_distance <= 0.0f) ||
       (complete_tolerance <= 0.0f))
    {
        return 0u;
    }

    __disable_irq();
    if((s_state != PATH_EXECUTOR_IDLE) &&
       (s_state != PATH_EXECUTOR_DONE) &&
       (s_state != PATH_EXECUTOR_FAULT))
    {
        __enable_irq();
        return 0u;
    }

    s_move_count = 1u;
    s_step_index = 0u;
    s_step_cell_count = 1u;
    s_step_distance = step_distance;
    s_step_complete_tolerance = complete_tolerance;
    s_fault_reason = PATH_EXECUTOR_FAULT_NONE;
    if(path_executor_prepare_body_step(direction) == 0u)
    {
        path_executor_enter_fault(PATH_EXECUTOR_FAULT_INVALID_MOVE);
        __enable_irq();
        return 0u;
    }

    s_state = PATH_EXECUTOR_RUN_STEP;
    path_executor_stop_motion();
    __enable_irq();
    return 1u;
}

static uint8_t path_executor_start_internal(const sokoban_solution_t *solution,
                                            float step_distance)
{
    uint16_t idx;

    if((solution == 0) || (solution->solved == 0u) ||
       (solution->move_count == 0u) || (step_distance <= 0.0f))
    {
        path_executor_stop_motion();
        return 0u;
    }

    __disable_irq();
    if((s_state != PATH_EXECUTOR_IDLE) && (s_state != PATH_EXECUTOR_DONE) && (s_state != PATH_EXECUTOR_FAULT))
    {
        __enable_irq();
        return 0u;
    }

    for(idx = 0u; (idx < solution->move_count) && (idx < SOKOBAN_MAX_MOVES); idx++)
    {
        s_move_seq[idx] = solution->move_seq[idx];
    }
    s_move_seq[idx] = '\0';
    s_move_count = idx;
    s_step_index = 0u;
    s_step_cell_count = 1u;
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_step_distance = step_distance;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
    s_settle_elapsed_ticks = 0u;
    s_fault_reason = PATH_EXECUTOR_FAULT_NONE;
    s_step_axis = PATH_AXIS_VX;
    s_state = PATH_EXECUTOR_LOAD_STEP;
    path_executor_stop_motion();
    __enable_irq();
    return (uint8_t)(s_move_count > 0u);
}

// 开始执行路径
uint8_t path_executor_start(const sokoban_solution_t *solution)
{
    return path_executor_start_internal(solution, PATH_EXECUTOR_CELL_DISTANCE);
}

uint8_t path_executor_start_with_distance(const sokoban_solution_t *solution, float step_distance)
{
    return path_executor_start_internal(solution, step_distance);
}

// 10ms 更新函数
void path_executor_update_10ms(void)
{
    float error_x;
    float error_y;
    float measured_vx;
    float measured_vy;
    float yaw_error;
    uint8_t within_tolerance_x;
    uint8_t within_tolerance_y;
    uint8_t within_tolerance;

    switch(s_state)
    {
        case PATH_EXECUTOR_IDLE:  // 空闲
        case PATH_EXECUTOR_DONE:  // 完成
        case PATH_EXECUTOR_FAULT: // 故障
        {
            path_executor_stop_motion();
            break;
        }

        case PATH_EXECUTOR_LOAD_STEP:  // 加载步骤
        {
            char move;

            if(s_step_index >= s_move_count)
            {
                s_state = PATH_EXECUTOR_DONE;
                path_executor_stop_motion();
                break;
            }

            move = s_move_seq[s_step_index];
            s_step_cell_count = 1u;
            while(((uint16_t)(s_step_index + s_step_cell_count) < s_move_count) &&
                  (s_move_seq[s_step_index + s_step_cell_count] == move))
            {
                s_step_cell_count++;
            }

            if(!path_executor_prepare_step(move))
            {
                path_executor_enter_fault(PATH_EXECUTOR_FAULT_INVALID_MOVE);
                break;
            }

            s_state = PATH_EXECUTOR_RUN_STEP;
            break;
        }

        case PATH_EXECUTOR_RUN_STEP:  // 执行步骤
        {
            s_step_position_x += path_executor_measure_vx() * PATH_EXECUTOR_DT_S;
            s_step_position_y += path_executor_measure_vy() * PATH_EXECUTOR_DT_S;
            s_step_elapsed_ticks++;

            error_x = s_step_target_x - s_step_position_x;
            error_y = s_step_target_y - s_step_position_y;
            within_tolerance = path_executor_within_tolerance(
                error_x, error_y, &within_tolerance_x, &within_tolerance_y);

            if(within_tolerance)
            {
                path_executor_stop_motion();
                s_complete_ticks++;
            }
            else
            {
                s_complete_ticks = 0u;
                PositionControl(s_step_target_x,
                                s_step_target_y,
                                s_step_position_x,
                                s_step_position_y);
                if(within_tolerance_x) target_vx = 0.0f;
                if(within_tolerance_y) target_vy = 0.0f;
            }

            if(s_complete_ticks >= PATH_EXECUTOR_COMPLETE_TICKS)
            {
                path_executor_stop_motion();
                s_settle_ticks = 0u;
                s_settle_elapsed_ticks = 0u;
                s_state = PATH_EXECUTOR_SETTLE;
                MecanumSpeedPidReset();
            }
            else if(s_step_elapsed_ticks >=
                    (uint32_t)PATH_EXECUTOR_TIMEOUT_TICKS * s_step_cell_count)
            {
                path_executor_enter_fault(PATH_EXECUTOR_FAULT_RUN_TIMEOUT);
            }
            break;
        }

        case PATH_EXECUTOR_SETTLE:  // 稳定状态
        {
            path_executor_stop_motion();
            measured_vx = path_executor_measure_vx();
            measured_vy = path_executor_measure_vy();
            yaw_error = path_executor_get_yaw_error();
            s_step_position_x += measured_vx * PATH_EXECUTOR_DT_S;
            s_step_position_y += measured_vy * PATH_EXECUTOR_DT_S;
            s_settle_elapsed_ticks++;

            if(path_executor_body_settled(measured_vx, measured_vy, yaw_error))
            {
                s_settle_ticks++;
            }
            else
            {
                s_settle_ticks = 0u;
            }

            if(s_settle_ticks >= PATH_EXECUTOR_SETTLE_TICKS)
            {
                error_x = s_step_target_x - s_step_position_x;
                error_y = s_step_target_y - s_step_position_y;
                within_tolerance = path_executor_within_tolerance(
                    error_x, error_y, &within_tolerance_x, &within_tolerance_y);

                if(!within_tolerance)
                {
                    s_complete_ticks = 0u;
                    s_settle_ticks = 0u;
                    s_settle_elapsed_ticks = 0u;
                    s_state = PATH_EXECUTOR_RUN_STEP;
                }
                else
                {
                    s_carry_error_x = error_x;
                    s_carry_error_y = error_y;
                    s_step_index = (uint16_t)(s_step_index +
                                              s_step_cell_count);
                    if(s_step_index >= s_move_count)
                    {
                        s_state = PATH_EXECUTOR_DONE;
                    }
                    else
                    {
                        s_state = PATH_EXECUTOR_LOAD_STEP;
                    }
                }
            }
            else if(s_settle_elapsed_ticks >= PATH_EXECUTOR_SETTLE_TIMEOUT)
            {
                if((fabsf(yaw_error) > PATH_EXECUTOR_SETTLE_YAW) ||
                   (fabsf(Gyro.x) > PATH_EXECUTOR_SETTLE_RATE))
                {
                    path_executor_enter_fault(PATH_EXECUTOR_FAULT_YAW_TIMEOUT);
                }
                else
                {
                    path_executor_enter_fault(PATH_EXECUTOR_FAULT_STOP_TIMEOUT);
                }
            }
            break;
        }

        default:  // 默认情况，设为故障
        {
            path_executor_enter_fault(PATH_EXECUTOR_FAULT_INVALID_MOVE);
            break;
        }
    }
}

// 检查是否空闲
uint8_t path_executor_is_idle(void)
{
    return (uint8_t)((s_state == PATH_EXECUTOR_IDLE) ||
                     (s_state == PATH_EXECUTOR_DONE) ||
                     (s_state == PATH_EXECUTOR_FAULT));
}

// 检查是否完成
uint8_t path_executor_is_done(void)
{
    return (uint8_t)(s_state == PATH_EXECUTOR_DONE);
}

uint8_t path_executor_is_fault(void)
{
    return (uint8_t)(s_state == PATH_EXECUTOR_FAULT);
}

uint8_t path_executor_is_running(void)
{
    return (uint8_t)(s_state == PATH_EXECUTOR_RUN_STEP);
}

void path_executor_set_heading(sokoban_direction_t heading)
{
    if((uint8_t)heading <= (uint8_t)SOKOBAN_DIR_LEFT)
    {
        uint8_t delta;
        float carry_x;
        float carry_y;

        __disable_irq();
        delta = (uint8_t)(((uint8_t)heading + 4u - (uint8_t)s_heading) & 3u);
        carry_x = s_carry_error_x;
        carry_y = s_carry_error_y;

        if(delta == 1u)
        {
            s_carry_error_x = -carry_y;
            s_carry_error_y = carry_x;
        }
        else if(delta == 2u)
        {
            s_carry_error_x = -carry_x;
            s_carry_error_y = -carry_y;
        }
        else if(delta == 3u)
        {
            s_carry_error_x = carry_y;
            s_carry_error_y = -carry_x;
        }
        s_heading = heading;
        __enable_irq();
    }
}

float path_executor_get_last_step_distance(void)
{
    return fabsf((s_step_axis == PATH_AXIS_VX) ? s_step_position_x : s_step_position_y);
}

uint8_t path_executor_get_state(void)
{
    return (uint8_t)s_state;
}

path_executor_fault_reason_t path_executor_get_fault_reason(void)
{
    return s_fault_reason;
}

uint16_t path_executor_get_step_index(void)
{
    return s_step_index;
}

uint16_t path_executor_get_move_count(void)
{
    return s_move_count;
}

float path_executor_get_position_x(void)
{
    return s_step_position_x;
}

float path_executor_get_position_y(void)
{
    return s_step_position_y;
}

float path_executor_get_target_x(void)
{
    return s_step_target_x;
}

float path_executor_get_target_y(void)
{
    return s_step_target_y;
}

float path_executor_get_carry_x(void)
{
    return s_carry_error_x;
}

float path_executor_get_carry_y(void)
{
    return s_carry_error_y;
}
