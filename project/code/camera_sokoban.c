#include "zf_common_headfile.h"
#include "isr.h"
#include "camera_sokoban.h"

extern volatile uint32_t camera_frame_sequence;

// 屏幕显示相关宏定义
#define SCREEN_TEXT_X                (0u)       // 屏幕文本X坐标
#define SCREEN_LINE0_Y               (0u)       // 屏幕第0行Y坐标
#define SCREEN_LINE1_Y               (20u)      // 屏幕第1行Y坐标
#define SCREEN_LINE2_Y               (40u)      // 屏幕第2行Y坐标
#define SCREEN_LINE3_Y               (60u)      // 屏幕第3行Y坐标
#define SCREEN_LINE4_Y               (80u)      // 屏幕第4行Y坐标
#define SCREEN_LINE5_Y               (100u)     // 屏幕第5行Y坐标
#define SCREEN_LINE6_Y               (120u)     // 屏幕第6行Y坐标
#define SCREEN_LINE7_Y               (140u)     // 屏幕第7行Y坐标
#define SCREEN_LINE8_Y               (160u)     // 屏幕第8行Y坐标
#define SCREEN_LINE9_Y               (180u)     // 屏幕第9行Y坐标
#define SCREEN_LINE10_Y              (200u)     // 屏幕第10行Y坐标
#define SCREEN_PATH_CHARS_PER_LINE   (28u)  // 每行路径字符数
#define SCREEN_PATH_LINE_COUNT       (6u)   // 路径显示行数
#define SCREEN_PATH_PREVIEW_LEN      (SCREEN_PATH_CHARS_PER_LINE * SCREEN_PATH_LINE_COUNT)  // 路径预览长度
#define CN_PATH_BUFFER_SIZE          (SOKOBAN_MAX_MOVES * 3u + 1u)  // 中文路径缓冲区大小

// 摄像机帧快照结构体
typedef struct
{
    uint8 valid;                                    // 是否有效
    uint32 sequence;
    uint8 player_row;                               // 玩家行位置
    uint8 player_col;                               // 玩家列位置
    uint8 raw_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_WIDTH];  // 原始地图数据
} camera_frame_snapshot_t;

// 静态变量声明
static camera_frame_snapshot_t s_last_camera_frame;              // 最后一帧摄像机数据
static sokoban_solution_t s_last_solution;                       // 最后一个解
static sokoban_status_t s_last_status = SOKOBAN_STATUS_INVALID_MAP;  // 最后状态
static char s_last_solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];  // 求解器地图
static char s_path_cn_buffer[CN_PATH_BUFFER_SIZE];               // 中文路径缓冲区
static uint8 s_fixed_test_map_processed = 0u;                    // 固定测试地图是否已处理
static uint8 s_solution_pending = 0u;                            // 是否有待处理的解

// 固定测试地图
static const char s_fixed_test_solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
	"################",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#------$-..----#",
	"#@-------------#",
	"#------$-------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

// 初始化求解器地图缓冲区
static void initialize_solver_map_buffer(char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8 row;
    uint8 col;

    // 将整个缓冲区初始化为'?'
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            solver_map[row][col] = '?';
        }
        solver_map[row][SOKOBAN_MAP_WIDTH] = '\0';  // 字符串终止符
    }
}

// 存储当前摄像机帧
static void snapshot_current_camera_frame(camera_frame_snapshot_t *snapshot)
{
    uint8 row;
    uint8 col;
    uint32_t sequence_before;
    uint32_t sequence_after;

    do
    {
        sequence_before = camera_frame_sequence;
        if((sequence_before == 0u) || ((sequence_before & 1u) != 0u))
        {
            sequence_after = sequence_before + 1u;
            continue;
        }
        snapshot->player_row = (uint8)car_y;
        snapshot->player_col = (uint8)car_x;
        for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
        {
            for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
            {
                snapshot->raw_map[row][col] = (uint8)photo_data[row][col];
            }
        }
        sequence_after = camera_frame_sequence;
    } while((sequence_before != sequence_after) || ((sequence_after & 1u) != 0u));
    snapshot->valid = 1u;
    snapshot->sequence = sequence_after;
}

// 检查摄像机帧是否已改变
static uint8 camera_frame_changed(const camera_frame_snapshot_t *current)
{
    uint8 row;
    uint8 col;

    // 如果没有先前的帧，则当前帧被视为已更改
    if(!s_last_camera_frame.valid)
    {
        return 1u;
    }

    // 检查玩家位置是否改变
    if((current->player_row != s_last_camera_frame.player_row) ||
       (current->player_col != s_last_camera_frame.player_col))
    {
        return 1u;
    }

    // 检查地图内容是否改变
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(current->raw_map[row][col] != s_last_camera_frame.raw_map[row][col])
            {
                return 1u;
            }
        }
    }

    return 0u;  // 没有改变
}

// 不帀处理摄像机帧
static void cache_camera_frame(const camera_frame_snapshot_t *snapshot)
{
    uint8 row;
    uint8 col;

    // 复制快照信息到静态缓冲区
    s_last_camera_frame.valid = snapshot->valid;
    s_last_camera_frame.player_row = snapshot->player_row;
    s_last_camera_frame.player_col = snapshot->player_col;

    // 复制地图数据
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            s_last_camera_frame.raw_map[row][col] = snapshot->raw_map[row][col];
        }
    }
}

// 使用整数值转换为求解器使用的字符
static sokoban_status_t camera_cell_to_solver_char(uint8 raw_value, char *solver_char)
{
    switch(raw_value)
    {
        case 0u:  // 路面
        {
            *solver_char = '-';
            return SOKOBAN_STATUS_OK;
        }

        case 1u:  // 墙壁
        {
            *solver_char = '#';
            return SOKOBAN_STATUS_OK;
        }

        case 2u:  // 目标位置
        {
            *solver_char = '.';
            return SOKOBAN_STATUS_OK;
        }

        case 3u:  // 箱子
        {
            *solver_char = '$';
            return SOKOBAN_STATUS_OK;
        }

        /* UART1 raw map value 5 means a bomb box. */
        case 5u:
        {
            *solver_char = '*';
            return SOKOBAN_STATUS_OK;
        }

        default:  // 未知值，地图无效
        {
            return SOKOBAN_STATUS_INVALID_MAP;
        }
    }
}

/*
 * 根据摄像头数据构建求解器地图
 * 摄像头编码:
 * 0 = 路面
 * 1 = 墙壁
 * 2 = 目标位置
 * 3 = 箱子
 * 玩家位置来自 car_x/car_y
 */
static sokoban_status_t build_solver_map_from_camera(const camera_frame_snapshot_t *snapshot,
                                                     char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8 row;
    uint8 col;
    uint8 solver_player_row;

    initialize_solver_map_buffer(solver_map);

    if((snapshot->player_row >= SOKOBAN_MAP_HEIGHT) || (snapshot->player_col >= SOKOBAN_MAP_WIDTH))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    solver_player_row = snapshot->player_row;

    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            sokoban_status_t convert_status = camera_cell_to_solver_char(snapshot->raw_map[row][col], &solver_map[row][col]);
            if(convert_status != SOKOBAN_STATUS_OK)
            {
                return convert_status;
            }
        }
        solver_map[row][SOKOBAN_MAP_WIDTH] = '\0';
    }

    if('#' == solver_map[solver_player_row][snapshot->player_col])
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    if(('$' == solver_map[solver_player_row][snapshot->player_col]) ||
       ('*' == solver_map[solver_player_row][snapshot->player_col]))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    if('.' == solver_map[solver_player_row][snapshot->player_col])
    {
        solver_map[solver_player_row][snapshot->player_col] = '+';
    }
    else
    {
        solver_map[solver_player_row][snapshot->player_col] = '@';
    }

    return SOKOBAN_STATUS_OK;
}

// 复制求解器地图
static void copy_solver_map(char dst[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                            const char src[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8 row;
    uint8 col;

    // 逐行逐列复制地图
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_STRIDE; col++)
        {
            dst[row][col] = src[row][col];
        }
    }
}

// 在求解器地图中找到玩家位置
static uint8 find_player_in_solver_map(const char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                       uint8 *player_row,
                                       uint8 *player_col)
{
    uint8 row;
    uint8 col;

    // 遍历地图寻找玩家符号 '@' 或 '+'
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(('@' == solver_map[row][col]) || ('+' == solver_map[row][col]))
            {
                *player_row = row;
                *player_col = col;
                return 1u;  // 找到玩家
            }
        }
    }

    return 0u;  // 未找到玩家
}

// 显示路径预览行
static void display_path_preview_lines(const char *path, uint16 move_count)
{
    // 路径显示行Y坐标
    static const uint8 s_path_line_y[SCREEN_PATH_LINE_COUNT] =
    {
        SCREEN_LINE5_Y,
        SCREEN_LINE6_Y,
        SCREEN_LINE7_Y,
        SCREEN_LINE8_Y,
        SCREEN_LINE9_Y,
        SCREEN_LINE10_Y
    };

    char line_buffer[SCREEN_PATH_CHARS_PER_LINE + 1u];
    uint16 line_index;
    uint16 copied_count = 0u;

    // 逐行显示路径
    for(line_index = 0u; line_index < SCREEN_PATH_LINE_COUNT; line_index++)
    {
        uint16 char_index = 0u;

        while((char_index < SCREEN_PATH_CHARS_PER_LINE) && (copied_count < move_count))
        {
            line_buffer[char_index++] = path[copied_count++];
        }
        line_buffer[char_index] = '\0';

        if((move_count > SCREEN_PATH_PREVIEW_LEN) &&
           ((SCREEN_PATH_LINE_COUNT - 1u) == line_index) &&
           (char_index >= 3u))
        {
            line_buffer[char_index - 3u] = '.';
            line_buffer[char_index - 2u] = '.';
            line_buffer[char_index - 1u] = '.';
        }

        if(0u == char_index)
        {
            ips200_show_string(SCREEN_TEXT_X, s_path_line_y[line_index], " ");
        }
        else
        {
            ips200_show_string(SCREEN_TEXT_X, s_path_line_y[line_index], line_buffer);
        }
    }
}

// 显示求解器结果
static void display_solver_result(sokoban_status_t status,
                                  const sokoban_solution_t *solution,
                                  uint8 player_row,
                                  uint8 player_col)
{
    char line_buffer[64];

    ips200_clear();

    sprintf(line_buffer, "status:%s", sokoban_status_string(status));
    ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE0_Y, line_buffer);

    sprintf(line_buffer, "player:%u,%u", player_row, player_col);
    ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE1_Y, line_buffer);

    sprintf(line_buffer, "moves:%u push:%u", solution->move_count, solution->push_count);
    ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE2_Y, line_buffer);

    sprintf(line_buffer, "expand:%u/%u", solution->expanded_forward, solution->expanded_reverse);
    ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE3_Y, line_buffer);

    if((status == SOKOBAN_STATUS_OK) && (solution->move_count > SCREEN_PATH_PREVIEW_LEN))
    {
        sprintf(line_buffer, "path:1-%u/%u", SCREEN_PATH_PREVIEW_LEN, solution->move_count);
    }
    else
    {
        sprintf(line_buffer, "path:%u", solution->move_count);
    }
    ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE4_Y, line_buffer);

    if(status == SOKOBAN_STATUS_OK)
    {
        display_path_preview_lines(solution->move_seq, solution->move_count);
    }
    else
    {
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE5_Y, "N/A");
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE6_Y, " ");
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE7_Y, " ");
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE8_Y, " ");
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE9_Y, " ");
        ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE10_Y, " ");
    }
}

// 打印求解器地图到串口
static void print_solver_map_to_serial(const char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8 row;

    printf("map:\r\n");
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        printf("%s\r\n", solver_map[row]);
    }
}

// 打印求解结果到串口
static void print_solver_result_to_serial(sokoban_status_t status,
                                          const sokoban_solution_t *solution,
                                          uint8 player_row,
                                          uint8 player_col,
                                          const char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    printf("sokoban status=%s player=(%u,%u)\r\n",
           sokoban_status_string(status),
           player_row,
           player_col);

    print_solver_map_to_serial(solver_map);

    printf("moves=%u pushes=%u expanded_f=%u expanded_r=%u\r\n",
           solution->move_count,
           solution->push_count,
           solution->expanded_forward,
           solution->expanded_reverse);

    if(status == SOKOBAN_STATUS_OK)
    {
        printf("path_udlr=%s\r\n", solution->move_seq);
        sokoban_moves_to_chinese(solution->move_seq,
                                 solution->move_count,
                                 s_path_cn_buffer,
                                 sizeof(s_path_cn_buffer));
        printf("path_cn=%s\r\n", s_path_cn_buffer);
    }
    else
    {
        printf("path_udlr=N/A\r\n");
        printf("path_cn=N/A\r\n");
    }
}

// 发布求解结果
static void publish_solver_result(sokoban_status_t status,
                                  const sokoban_solution_t *solution,
                                  uint8 player_row,
                                  uint8 player_col,
                                  const char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    copy_solver_map(s_last_solver_map, solver_map);
    s_last_solution = *solution;
    s_last_status = status;
    s_solution_pending = (uint8)((status == SOKOBAN_STATUS_OK) && (solution->move_count > 0u));

    display_solver_result(s_last_status,
                          &s_last_solution,
                          player_row,
                          player_col);
}

// 处理新摄像机帧
static void handle_new_camera_frame(void)
{
    camera_frame_snapshot_t current_frame;
    char current_solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
    sokoban_solution_t current_solution;
    sokoban_status_t status;
    uint8 solver_player_row = 0u;

    snapshot_current_camera_frame(&current_frame);
    if(!camera_frame_changed(&current_frame))
    {
        flag = 0;
        return;
    }

    status = build_solver_map_from_camera(&current_frame, current_solver_map);
    (void)memset(&current_solution, 0, sizeof(current_solution));

    if(status == SOKOBAN_STATUS_OK)
    {
        status = sokoban_solve_decomposed(current_solver_map, &current_solution);
        if(status != SOKOBAN_STATUS_OK)
        {
            status = sokoban_solve_bidirectional_astar(current_solver_map, &current_solution);
        }
    }

    if(current_frame.player_row < SOKOBAN_MAP_HEIGHT)
    {
        solver_player_row = current_frame.player_row;
    }

    cache_camera_frame(&current_frame);
    publish_solver_result(status,
                          &current_solution,
                          solver_player_row,
                          current_frame.player_col,
                          current_solver_map);
		
//		for(int i=0;i<12;i++)
//		{
//			for(int j=0;j<16;j++)
//			{
//				ips200_show_uint(15*j,20*i,photo_data[i][j],1);
//			}
//		}

    flag = 0;
}

// 处理固定测试地图
static void handle_fixed_test_map(void)
{
    char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
    sokoban_solution_t current_solution;
    sokoban_status_t status = SOKOBAN_STATUS_OK;
    uint8 player_row = 0u;
    uint8 player_col = 0u;

    if(s_fixed_test_map_processed)
    {
        return;
    }

    copy_solver_map(solver_map, s_fixed_test_solver_map);
    (void)memset(&current_solution, 0, sizeof(current_solution));

    if(!find_player_in_solver_map(solver_map, &player_row, &player_col))
    {
        status = SOKOBAN_STATUS_INVALID_MAP;
    }

    if(status == SOKOBAN_STATUS_OK)
    {
        status = sokoban_solve_decomposed(solver_map, &current_solution);
        if(status != SOKOBAN_STATUS_OK)
        {
            status = sokoban_solve_bidirectional_astar(solver_map, &current_solution);
        }
    }

    publish_solver_result(status,
                          &current_solution,
                          player_row,
                          player_col,
                          solver_map);

    s_fixed_test_map_processed = 1u;
}

// 初始化摄像机索阻客 (sokoban)
void camera_sokoban_init(void)
{
    (void)memset(&s_last_camera_frame, 0, sizeof(s_last_camera_frame));
    (void)memset(&s_last_solution, 0, sizeof(s_last_solution));
    (void)memset(s_last_solver_map, 0, sizeof(s_last_solver_map));
    (void)memset(s_path_cn_buffer, 0, sizeof(s_path_cn_buffer));
    s_fixed_test_map_processed = 0u;
    s_solution_pending = 0u;
    s_last_status = SOKOBAN_STATUS_INVALID_MAP;

//     ips200_clear();
//     ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE0_Y, "camera sokoban");
// #if CAMERA_SOKOBAN_USE_FIXED_TEST_MAP
//     ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE1_Y, "fixed test mode");
//     printf("camera sokoban ready: fixed test mode\r\n");
// #else
//     ips200_show_string(SCREEN_TEXT_X, SCREEN_LINE1_Y, "waiting frame...");
//     printf("camera sokoban ready: camera mode\r\n");
// #endif
}

// 处理摄像头和索科班逻辑
void camera_sokoban_process(void)
{
#if CAMERA_SOKOBAN_USE_FIXED_TEST_MAP
    handle_fixed_test_map();
#else

    if(flag)
    {
        handle_new_camera_frame();
    }
#endif
}

// 获取并清除最后一个解决方案
uint8_t camera_sokoban_take_last_solution(sokoban_solution_t *out)
{
    if((out == 0) || (s_solution_pending == 0u) || (s_last_status != SOKOBAN_STATUS_OK))
    {
        return 0u;
    }

    *out = s_last_solution;
    s_solution_pending = 0u;
    return 1u;
}

// 获取最后一个解决方案（不清除）
const sokoban_solution_t *camera_sokoban_get_last_solution(void)
{
    return &s_last_solution;
}

// 获取最后一个求解状态
sokoban_status_t camera_sokoban_get_last_status(void)
{
    return s_last_status;
}

uint8_t camera_sokoban_copy_latest_frame(
    char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t *player_row,
    uint8_t *player_col,
    uint8_t *map_all_zero,
    uint32_t *sequence)
{
    camera_frame_snapshot_t snapshot;
    uint8 row;
    uint8 col;
    uint8 all_zero = 1u;

    if((solver_map == 0) || (player_row == 0) || (player_col == 0) ||
       (map_all_zero == 0) || (sequence == 0) || (camera_frame_sequence == 0u))
    {
        return 0u;
    }
    snapshot_current_camera_frame(&snapshot);
    for(row = 1u; row < SOKOBAN_MAP_HEIGHT-1; row++)
    {
        for(col = 1u; col < SOKOBAN_MAP_WIDTH-1; col++)
        {
            if(snapshot.raw_map[row][col] != 0u)
            {
                all_zero = 0u;
            }
        }
    }
    if(build_solver_map_from_camera(&snapshot, solver_map) != SOKOBAN_STATUS_OK)
    {
        return 0u;
    }
    *player_row = snapshot.player_row;
    *player_col = snapshot.player_col;
    *map_all_zero = all_zero;
    *sequence = snapshot.sequence;
    return 1u;
}

uint8_t camera_sokoban_copy_latest_pose(camera_pose_snapshot_t *pose)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if((pose == 0) || (camera_pose_sequence == 0u))
    {
        return 0u;
    }

    do
    {
        sequence_before = camera_pose_sequence;
        if((sequence_before == 0u) || ((sequence_before & 1u) != 0u))
        {
            sequence_after = sequence_before + 1u;
            continue;
        }
        pose->valid = camera_pose_valid;
        pose->confidence = camera_pose_confidence;
        pose->frame_sequence = camera_pose_frame_sequence;
        pose->x100 = camera_pose_x100;
        pose->y100 = camera_pose_y100;
        sequence_after = camera_pose_sequence;
    } while((sequence_before != sequence_after) ||
            ((sequence_after & 1u) != 0u));

    pose->sequence = sequence_after;
    return 1u;
}
