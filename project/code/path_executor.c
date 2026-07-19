#include "zf_common_headfile.h"
#include <math.h>
#include "path_executor.h"
#include "Mecnum.h"

// 路径执行器相关宏定义
#define PATH_EXECUTOR_DT_S            (0.01f)     //时间步长（秒）
#define PATH_EXECUTOR_CELL_DISTANCE  (23.0f)      // 每个单元格的距离
#define PATH_EXECUTOR_STEP_SPEED     (30.0f)      // 步进速度
#define PATH_EXECUTOR_COMPLETE_TOL   (1.0f)       // 完成容差
#define PATH_EXECUTOR_SETTLE_TICKS   (5u)         // 稳定 ticks 数
#define PATH_EXECUTOR_FIXED_YAW      (90.0f)       // 固定偏航角



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
static float s_step_progress = 0.0f;                                 // 步骤进度
static float s_step_target_vx = 0.0f;                                // 目标X速度
static float s_step_target_vy = 0.0f;                                // 目标Y速度
static float s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;          // 步骤距离
static volatile uint8_t s_settle_ticks = 0u;                         // 稳定ticks计数
static path_step_axis_t s_step_axis = PATH_AXIS_VX;                  // 当前轴
static volatile sokoban_direction_t s_heading = SOKOBAN_DIR_RIGHT;

// 保持航向
static void path_executor_hold_heading(void)
{
    target_yaw = PATH_EXECUTOR_FIXED_YAW;
}

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

// 准备步骤
static uint8_t path_executor_prepare_step(char move)
{
    sokoban_direction_t world_direction;
    sokoban_body_direction_t body_direction;
    s_step_progress = 0.0f;
    s_settle_ticks = 0u;

    switch(move)
    {
        case 'U': world_direction = SOKOBAN_DIR_UP; break;
        case 'D': world_direction = SOKOBAN_DIR_DOWN; break;
        case 'L': world_direction = SOKOBAN_DIR_LEFT; break;
        case 'R': world_direction = SOKOBAN_DIR_RIGHT; break;
        default: return 0u;
    }
    body_direction = sokoban_world_to_body(s_heading, world_direction);
    switch(body_direction)
    {
        case SOKOBAN_BODY_FORWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_vx = PATH_EXECUTOR_STEP_SPEED; s_step_target_vy = 0.0f; break;
        case SOKOBAN_BODY_BACKWARD:
            s_step_axis = PATH_AXIS_VX; s_step_target_vx = -PATH_EXECUTOR_STEP_SPEED; s_step_target_vy = 0.0f; break;
        case SOKOBAN_BODY_LEFT:
            s_step_axis = PATH_AXIS_VY; s_step_target_vx = 0.0f; s_step_target_vy = PATH_EXECUTOR_STEP_SPEED; break;
        case SOKOBAN_BODY_RIGHT:
            s_step_axis = PATH_AXIS_VY; s_step_target_vx = 0.0f; s_step_target_vy = -PATH_EXECUTOR_STEP_SPEED; break;
        default: return 0u;
    }
    return 1u;
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
    s_step_progress = 0.0f;
    s_step_target_vx = 0.0f;
    s_step_target_vy = 0.0f;
    s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;
    s_settle_ticks = 0u;
    s_step_axis = PATH_AXIS_VX;
    s_state = PATH_EXECUTOR_IDLE;
    s_heading = SOKOBAN_DIR_UP;
    path_executor_stop_motion();
}

// 开始执行路径
uint8_t path_executor_start(const sokoban_solution_t *solution)
{
    uint16_t idx;

    if((solution == 0) || (solution->solved == 0u) || (solution->move_count == 0u))
    {
        path_executor_stop_motion();
        return 0u;
    }

    if((s_state != PATH_EXECUTOR_IDLE) && (s_state != PATH_EXECUTOR_DONE) && (s_state != PATH_EXECUTOR_FAULT))
    {
        return 0u;
    }

    __disable_irq();  // 禁用中断以确保原子操作
    for(idx = 0u; (idx < solution->move_count) && (idx < SOKOBAN_MAX_MOVES); idx++)
    {
        s_move_seq[idx] = solution->move_seq[idx];
    }
    s_move_seq[idx] = '\0';
    s_move_count = idx;
    s_step_index = 0u;
    s_step_progress = 0.0f;
    s_step_target_vx = 0.0f;
    s_step_target_vy = 0.0f;
    s_step_distance = PATH_EXECUTOR_CELL_DISTANCE;
    s_settle_ticks = 0u;
    s_step_axis = PATH_AXIS_VX;
    s_state = PATH_EXECUTOR_LOAD_STEP;
    __enable_irq();  // 重新启用中断

    path_executor_stop_motion();
    return (uint8_t)(s_move_count > 0u);
}

uint8_t path_executor_start_with_distance(const sokoban_solution_t *solution, float step_distance)
{
    uint8_t started;
    if(step_distance <= 0.0f)
    {
        return 0u;
    }
    started = path_executor_start(solution);
    if(started != 0u)
    {
        __disable_irq();
        s_step_distance = step_distance;
        __enable_irq();
    }
    return started;
}

// 10ms 更新函数
void path_executor_update_10ms(void)
{
    float axis_velocity;

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
            /* 使用位置环 PID 控制：将当前已行进距离作为反馈，目标为单步总距离 */
            if(s_step_axis == PATH_AXIS_VX)
            {
                PositionControl(s_step_distance, 0.0f, s_step_progress, 0.0f);
                axis_velocity = path_executor_measure_vx();
                s_step_progress += fabsf(axis_velocity) * PATH_EXECUTOR_DT_S;
            }
            else
            {
                PositionControl(0.0f, s_step_distance, 0.0f, s_step_progress);
                axis_velocity = path_executor_measure_vy();
                s_step_progress += fabsf(axis_velocity) * PATH_EXECUTOR_DT_S;
            }

            if(s_step_progress + PATH_EXECUTOR_COMPLETE_TOL >= s_step_distance)
            {
                path_executor_stop_motion();
                s_settle_ticks = 0u;
                s_state = PATH_EXECUTOR_SETTLE;
								MecanumSpeedPidReset();
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

void path_executor_set_heading(sokoban_direction_t heading)
{
    if((uint8_t)heading <= (uint8_t)SOKOBAN_DIR_LEFT)
    {
        s_heading = heading;
    }
}

float path_executor_get_last_step_distance(void)
{
    return s_step_progress;
}
