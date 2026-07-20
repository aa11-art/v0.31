#include "zf_common_headfile.h"
#include <math.h>
#include "path_executor.h"
#include "Mecnum.h"

// 路径执行器相关宏定义
#define PATH_EXECUTOR_DT_S            (0.01f)     //时间步长（秒）
#define PATH_EXECUTOR_CELL_DISTANCE  (24.0f)      // 每个单元格的距离
#define PATH_EXECUTOR_COMPLETE_TOL   (1.2f)       // 完成容差
#define PATH_EXECUTOR_COMPLETE_TICKS (8.0f)         // 连续到位 ticks 数
#define PATH_EXECUTOR_SETTLE_TICKS   (5u)         // 稳定 ticks 数
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

// 静态变量声明
static volatile path_executor_state_t s_state = PATH_EXECUTOR_IDLE;  // 当前状态
static char s_move_seq[SOKOBAN_MAX_MOVES + 1u];                      // 移动序列
static uint16_t s_move_count = 0u;                                   // 移动计数
static volatile uint16_t s_step_index = 0u;                          // 当前步骤索引
static float s_step_position_x = 0.0f;                               // 单步X位置
static float s_step_position_y = 0.0f;                               // 单步Y位置
static float s_step_target_x = 0.0f;                                 // 单步X目标
static float s_step_target_y = 0.0f;                                 // 单步Y目标
static float s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;          // 步骤距离
static float s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL; // 运动轴完成容差
static volatile uint16_t s_step_elapsed_ticks = 0u;                  // 单步运行 ticks 数
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

// 测量X轴速度
static float path_executor_measure_vx(void)
{
    return (float)(encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f;
}

// 测量Y轴速度
static float path_executor_measure_vy(void)
{
    return (float)(-encoder_fl + encoder_fr + encoder_bl - encoder_br) / 4.0f;
}

static uint8_t path_executor_prepare_body_step(sokoban_body_direction_t body_direction)
{
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;

    switch(body_direction)
    {
        case SOKOBAN_BODY_FORWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_x = s_step_distance; break;
        case SOKOBAN_BODY_BACKWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_x = -s_step_distance; break;
        case SOKOBAN_BODY_LEFT:
            s_step_axis = PATH_AXIS_VY; s_step_target_y = s_step_distance; break;
        case SOKOBAN_BODY_RIGHT:
            s_step_axis = PATH_AXIS_VY; s_step_target_y = -s_step_distance; break;
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
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
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
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
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
    s_step_distance = step_distance;
    s_step_complete_tolerance = complete_tolerance;
    if(path_executor_prepare_body_step(direction) == 0u)
    {
        s_state = PATH_EXECUTOR_FAULT;
        path_executor_stop_motion();
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
    s_step_position_x = 0.0f;
    s_step_position_y = 0.0f;
    s_step_target_x = 0.0f;
    s_step_target_y = 0.0f;
    s_step_distance = step_distance;
    s_step_complete_tolerance = PATH_EXECUTOR_COMPLETE_TOL;
    s_step_elapsed_ticks = 0u;
    s_complete_ticks = 0u;
    s_settle_ticks = 0u;
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
            if(s_step_index >= s_move_count)
            {
                s_state = PATH_EXECUTOR_DONE;
                path_executor_stop_motion();
                break;
            }

            if(!path_executor_prepare_step(s_move_seq[s_step_index]))
            {
                s_state = PATH_EXECUTOR_FAULT;
                path_executor_stop_motion();
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
            if(s_step_axis == PATH_AXIS_VX)
            {
                within_tolerance_x =
                    (uint8_t)(fabsf(error_x) <= s_step_complete_tolerance);
                within_tolerance_y =
                    (uint8_t)(fabsf(error_y) <= PATH_EXECUTOR_COMPLETE_TOL);
            }
            else
            {
                within_tolerance_x =
                    (uint8_t)(fabsf(error_x) <= PATH_EXECUTOR_COMPLETE_TOL);
                within_tolerance_y =
                    (uint8_t)(fabsf(error_y) <= s_step_complete_tolerance);
            }
            within_tolerance =
                (uint8_t)(within_tolerance_x && within_tolerance_y);

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
                s_state = PATH_EXECUTOR_SETTLE;
                MecanumSpeedPidReset();
            }
            else if(s_step_elapsed_ticks >= PATH_EXECUTOR_TIMEOUT_TICKS)
            {
                path_executor_stop_motion();
                MecanumSpeedPidReset();
                s_state = PATH_EXECUTOR_FAULT;
            }
            break;
        }

        case PATH_EXECUTOR_SETTLE:  // 稳定状态
        {
            path_executor_stop_motion();
            s_settle_ticks++;

            if(s_settle_ticks >= PATH_EXECUTOR_SETTLE_TICKS)
            {
                s_step_index++;
                if(s_step_index >= s_move_count)
                {
                    s_state = PATH_EXECUTOR_DONE;
                }
                else
                {
                    s_state = PATH_EXECUTOR_LOAD_STEP;
                    
                }
            }
            break;
        }

        default:  // 默认情况，设为故障
        {
            s_state = PATH_EXECUTOR_FAULT;
            path_executor_stop_motion();
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
        s_heading = heading;
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
