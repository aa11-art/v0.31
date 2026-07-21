#include "zf_common_headfile.h"
#include <string.h>
#include "camera_sokoban.h"
#include "path_executor.h"
#include "mission_controller.h"
#include "math.h"
#include "isr.h"

extern volatile uint32_t camera_frame_sequence;

#define TURN_YAW_SIGN                       (-1)
#define LABEL_REPLY_TIMEOUT_10MS            (50u)
#define LABEL_MAX_ATTEMPTS                  (4u)
#define LABEL_MAX_VALUE                     (10u)
#define DIGIT_RETRY_BACKOFF_DISTANCE        (3.0f)
#define DIGIT_RETRY_BACKOFF_TOLERANCE       (0.2f)
#define MISSION_BOOT_DELAY_10MS             (300u)
#define MISSION_MAP_WAIT_TIMEOUT_10MS        (1000u)
#define MISSION_RENDER_TIMEOUT_10MS          (300u)
#define MISSION_TURN_TIMEOUT_10MS            (500u)
#define MISSION_ABORT_HOLD_10MS              (300u)
#define MISSION_MAP_STABLE_FRAMES            (3u)
#define MISSION_MAX_RECOVERIES                (3u)
#define MISSION_STILL_ENCODER_LIMIT          (2)
#define MISSION_PUSH_SETTLE_STILL_10MS        (20u)
#define MISSION_PUSH_SETTLE_TIMEOUT_10MS      (100u)

typedef struct
{
    uint8_t row;
    uint8_t col;
    uint8_t label;
    uint8_t active;
    uint8_t bomb;
} mission_tracked_box_t;

typedef struct
{
    uint8_t row;
    uint8_t col;
    uint8_t label;
    uint8_t active;
} mission_tracked_goal_t;

typedef struct
{
    uint8_t valid;
    uint8_t box_index;
    uint8_t goal_index;
    uint8_t is_bomb;
    uint8_t blast;
    uint8_t source_row;
    uint8_t source_col;
    uint8_t target_row;
    uint8_t target_col;
    uint8_t player_row;
    uint8_t player_col;
} mission_push_checkpoint_t;

static volatile mission_state_t s_state = MISSION_BOOT_DELAY;
static volatile uint8_t s_timer_running = 0u;
static volatile uint32_t s_elapsed_10ms = 0u;
static volatile uint32_t s_state_elapsed_10ms = 0u;
static volatile uint16_t s_abort_hold_10ms = 0u;
static volatile uint16_t s_push_settle_10ms = 0u;
static uint32_t s_last_frame_sequence = 0u;
static uint8_t s_frame_updated = 0u;
static uint8_t s_have_valid_map = 0u;
static uint8_t s_map_stable_frames = 0u;
static uint8_t s_player_row = 0u;
static uint8_t s_player_col = 0u;
static uint8_t s_candidate_player_row = 0u;
static uint8_t s_candidate_player_col = 0u;
static uint8_t s_entry_row = 0u;
static uint8_t s_entry_col = 0u;
static uint8_t s_blast_event_index = 0u;
static uint8_t s_blast_total = 0u;
static uint8_t s_segment_running = 0u;
static uint8_t s_turn_complete = 0u;
static uint8_t s_inspection_event = 0u;
static uint16_t s_inspection_move_index = 0u;
static uint8_t s_current_level = MISSION_FIRST_LEVEL;
static mission_level_result_t s_level_result = MISSION_LEVEL_RESULT_NONE;
static mission_level_result_t s_last_level_result = MISSION_LEVEL_RESULT_NONE;
static mission_fatal_fault_t s_fatal_fault = MISSION_FATAL_NONE;
static sokoban_direction_t s_heading = SOKOBAN_DIR_UP;
static sokoban_status_t s_last_status = SOKOBAN_STATUS_OK;
static float s_entry_distance = 24.0f;
static uint16_t s_task_move_index = 0u;
static uint16_t s_push_checkpoint_index = 0u;
static uint16_t s_push_checkpoint_total = 0u;
static uint8_t s_recovery_count = 0u;
static mission_recovery_status_t s_recovery_status = MISSION_RECOVERY_NONE;
static uint8_t s_tracking_initialized = 0u;
static uint8_t s_tracked_box_count = 0u;
static uint8_t s_tracked_goal_count = 0u;
static uint8_t s_expected_player_row = 0u;
static uint8_t s_expected_player_col = 0u;
static uint8_t s_predicted_player_row = 0u;
static uint8_t s_predicted_player_col = 0u;

static char s_current_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_candidate_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_initial_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_expected_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_predicted_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static sokoban_inspection_plan_t s_inspection_plan;
static sokoban_label_table_t s_labels;
static sokoban_solution_t s_task_solution;
static sokoban_solution_t s_segment_solution;
static mission_tracked_box_t s_tracked_boxes[SOKOBAN_MAX_BOXES];
static mission_tracked_box_t s_predicted_boxes[SOKOBAN_MAX_BOXES];
static mission_tracked_goal_t s_tracked_goals[SOKOBAN_MAX_BOXES];
static mission_tracked_goal_t s_predicted_goals[SOKOBAN_MAX_BOXES];
static mission_push_checkpoint_t s_push_checkpoint;

uint8_t label = 0u;

static void mission_set_state(mission_state_t state)
{
    __disable_irq();
    s_state = state;
    s_state_elapsed_10ms = 0u;
    __enable_irq();
}

static void mission_solution_reset(sokoban_solution_t *solution)
{
    uint8_t idx;
    (void)memset(solution, 0, sizeof(*solution));
    for(idx = 0u; idx < SOKOBAN_MAX_BOMBS; idx++)
    {
        solution->blast_move_indices[idx] = SOKOBAN_INVALID_MOVE_INDEX;
        solution->blast_rows[idx] = SOKOBAN_INVALID_CELL;
        solution->blast_cols[idx] = SOKOBAN_INVALID_CELL;
    }
}

static uint8_t mission_encoder_is_still(void)
{
    return (uint8_t)((encoder_fl >= -MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_fl <= MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_fr >= -MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_fr <= MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_bl >= -MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_bl <= MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_br >= -MISSION_STILL_ENCODER_LIMIT) &&
                     (encoder_br <= MISSION_STILL_ENCODER_LIMIT));
}

static void mission_reset_map_acquisition(void)
{
    s_have_valid_map = 0u;
    s_map_stable_frames = 0u;
    s_frame_updated = 0u;
    s_last_frame_sequence = camera_frame_sequence;
    (void)memset(s_candidate_map, 0, sizeof(s_candidate_map));
}

static void mission_reset_level_data(void)
{
    s_level_result = MISSION_LEVEL_RESULT_NONE;
    mission_reset_map_acquisition();
    s_blast_event_index = 0u;
    s_blast_total = 0u;
    s_segment_running = 0u;
    s_turn_complete = 0u;
    s_inspection_event = 0u;
    s_inspection_move_index = 0u;
    s_task_move_index = 0u;
    s_push_checkpoint_index = 0u;
    s_push_checkpoint_total = 0u;
    s_push_settle_10ms = 0u;
    s_recovery_count = 0u;
    s_recovery_status = MISSION_RECOVERY_NONE;
    s_tracking_initialized = 0u;
    s_tracked_box_count = 0u;
    s_tracked_goal_count = 0u;
    label = 0u;
    (void)memset(&s_labels, 0, sizeof(s_labels));
    (void)memset(&s_inspection_plan, 0, sizeof(s_inspection_plan));
    (void)memset(&s_push_checkpoint, 0, sizeof(s_push_checkpoint));
    (void)memset(s_tracked_boxes, 0, sizeof(s_tracked_boxes));
    (void)memset(s_predicted_boxes, 0, sizeof(s_predicted_boxes));
    (void)memset(s_tracked_goals, 0, sizeof(s_tracked_goals));
    (void)memset(s_predicted_goals, 0, sizeof(s_predicted_goals));
    mission_solution_reset(&s_task_solution);
    mission_solution_reset(&s_segment_solution);
}

static void mission_enter_fatal(mission_fatal_fault_t fault,
                                sokoban_status_t status)
{
    s_last_status = status;
    s_fatal_fault = fault;
    s_timer_running = 0u;
    path_executor_abort();
    MecanumCarStop();
    mission_set_state(MISSION_FAULT);
}

static uint8_t mission_map_structurally_valid(
    const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t row,
    uint8_t col)
{
    uint8_t normal_box_count = 0u;
    uint8_t bomb_count = 0u;
    uint8_t goal_count = 0u;
    uint8_t y;
    uint8_t x;

    if((row >= SOKOBAN_MAP_HEIGHT) || (col >= SOKOBAN_MAP_WIDTH)) return 0u;
    for(y = 0u; y < SOKOBAN_MAP_HEIGHT; y++)
    {
        for(x = 0u; x < SOKOBAN_MAP_WIDTH; x++)
        {
            if(map[y][x] == '$') normal_box_count++;
            else if(map[y][x] == '*') bomb_count++;
            else if((map[y][x] == '.') || (map[y][x] == 'G') ||
                    (map[y][x] == '+')) goal_count++;
        }
    }

    if((s_current_level == 1u) && (bomb_count != 0u)) return 0u;
    return (uint8_t)((goal_count > 0u) &&
                     (normal_box_count == goal_count) &&
                     (bomb_count <= SOKOBAN_MAX_BOMBS) &&
                     ((uint8_t)(normal_box_count + bomb_count) <=
                      SOKOBAN_MAX_BOXES));
}

static uint8_t mission_task_complete(
    const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8_t row;
    uint8_t col;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = map[row][col];
            if((ch == '$') || (ch == '.') || (ch == 'G') || (ch == '+'))
            {
                return 0u;
            }
        }
    }
    return 1u;
}

static uint8_t mission_abs_diff(uint8_t lhs, uint8_t rhs)
{
    return (uint8_t)(lhs > rhs ? lhs - rhs : rhs - lhs);
}

static uint8_t mission_cells_near(uint8_t row_a, uint8_t col_a,
                                  uint8_t row_b, uint8_t col_b)
{
    return (uint8_t)((mission_abs_diff(row_a, row_b) <= 1u) &&
                     (mission_abs_diff(col_a, col_b) <= 1u));
}

static void mission_normalize_map(
    const char source[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    char destination[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8_t row;
    uint8_t col;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = source[row][col];
            destination[row][col] = (ch == '@') ? '-' :
                                    (((ch == '+') || (ch == 'G')) ? '.' : ch);
        }
        destination[row][SOKOBAN_MAP_WIDTH] = '\0';
    }
}

static uint8_t mission_boundary_is_intact(
    const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8_t row;
    uint8_t col;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(((row == 0u) || (col == 0u) ||
                (row + 1u == SOKOBAN_MAP_HEIGHT) ||
                (col + 1u == SOKOBAN_MAP_WIDTH)) &&
               (map[row][col] != '#')) return 0u;
        }
    }
    return 1u;
}

static uint8_t mission_checkpoint_map_valid(
    const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t player_row,
    uint8_t player_col)
{
    uint8_t normal_boxes = 0u;
    uint8_t bombs = 0u;
    uint8_t goals = 0u;
    uint8_t row;
    uint8_t col;
    char player_cell;

    if((player_row >= SOKOBAN_MAP_HEIGHT) ||
       (player_col >= SOKOBAN_MAP_WIDTH) ||
       !mission_boundary_is_intact(map)) return 0u;
    player_cell = map[player_row][player_col];
    if((player_cell == '#') || (player_cell == '$') ||
       (player_cell == '*')) return 0u;

    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = map[row][col];
            if(ch == '$') normal_boxes++;
            else if(ch == '*') bombs++;
            else if((ch == '.') || (ch == 'G') || (ch == '+')) goals++;
        }
    }
    if((s_current_level == 1u) && (bombs != 0u)) return 0u;
    return (uint8_t)((normal_boxes == goals) &&
                     (bombs <= SOKOBAN_MAX_BOMBS) &&
                     ((uint8_t)(normal_boxes + bombs) <= SOKOBAN_MAX_BOXES));
}

static void mission_build_solver_map(void)
{
    (void)memcpy(s_solver_map, s_expected_map, sizeof(s_solver_map));
    if(s_solver_map[s_expected_player_row][s_expected_player_col] == '.')
    {
        s_solver_map[s_expected_player_row][s_expected_player_col] = '+';
    }
    else
    {
        s_solver_map[s_expected_player_row][s_expected_player_col] = '@';
    }
}

static uint8_t mission_resolve_one_missing_label(void)
{
    uint8_t missing_count = 0u;
    uint8_t missing_is_box = 0u;
    uint8_t missing_index = 0u;
    uint8_t missing_label = 0u;
    uint8_t idx;

    for(idx = 0u; idx < s_labels.box_count; idx++)
    {
        if(s_labels.box_labels[idx] == SOKOBAN_BOMB_LABEL) continue;
        if(s_labels.box_labels[idx] == 0u)
        {
            missing_count++;
            missing_is_box = 1u;
            missing_index = idx;
        }
        else
        {
            missing_label ^= s_labels.box_labels[idx];
        }
    }
    for(idx = 0u; idx < s_labels.goal_count; idx++)
    {
        if(s_labels.goal_labels[idx] == 0u)
        {
            missing_count++;
            missing_is_box = 0u;
            missing_index = idx;
        }
        else
        {
            missing_label ^= s_labels.goal_labels[idx];
        }
    }
    if(missing_count == 0u) return 1u;
    if((missing_count != 1u) || (missing_label == 0u) ||
       (missing_label == SOKOBAN_BOMB_LABEL)) return 0u;
    if(missing_is_box != 0u)
        s_labels.box_labels[missing_index] = missing_label;
    else
        s_labels.goal_labels[missing_index] = missing_label;
    return 1u;
}

static uint8_t mission_initialize_tracking(uint8_t player_row,
                                           uint8_t player_col)
{
    uint8_t box_index = 0u;
    uint8_t goal_index = 0u;
    uint8_t row;
    uint8_t col;

    mission_normalize_map(s_initial_map, s_expected_map);
    s_expected_player_row = player_row;
    s_expected_player_col = player_col;
    if(s_current_level == 1u)
    {
        s_tracking_initialized = 1u;
        return 1u;
    }
    if(!mission_resolve_one_missing_label()) return 0u;
    (void)memset(s_tracked_boxes, 0, sizeof(s_tracked_boxes));
    (void)memset(s_tracked_goals, 0, sizeof(s_tracked_goals));
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = s_expected_map[row][col];
            if((ch == '$') || (ch == '*'))
            {
                if((box_index >= s_labels.box_count) ||
                   (box_index >= SOKOBAN_MAX_BOXES)) return 0u;
                s_tracked_boxes[box_index].row = row;
                s_tracked_boxes[box_index].col = col;
                s_tracked_boxes[box_index].label =
                    s_labels.box_labels[box_index];
                s_tracked_boxes[box_index].active = 1u;
                s_tracked_boxes[box_index].bomb = (uint8_t)(ch == '*');
                box_index++;
            }
            else if(ch == '.')
            {
                if((goal_index >= s_labels.goal_count) ||
                   (goal_index >= SOKOBAN_MAX_BOXES)) return 0u;
                s_tracked_goals[goal_index].row = row;
                s_tracked_goals[goal_index].col = col;
                s_tracked_goals[goal_index].label =
                    s_labels.goal_labels[goal_index];
                s_tracked_goals[goal_index].active = 1u;
                goal_index++;
            }
        }
    }
    if((box_index != s_labels.box_count) ||
       (goal_index != s_labels.goal_count)) return 0u;
    s_tracked_box_count = box_index;
    s_tracked_goal_count = goal_index;
    s_tracking_initialized = 1u;
    return 1u;
}

static uint8_t mission_initialize_labels(void)
{
    uint8_t row;
    uint8_t col;
    (void)memset(&s_labels, 0, sizeof(s_labels));
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = s_initial_map[row][col];
            if((ch == '$') || (ch == '*'))
            {
                if(s_labels.box_count >= SOKOBAN_MAX_BOXES) return 0u;
                if(ch == '*')
                {
                    s_labels.box_labels[s_labels.box_count] =
                        SOKOBAN_BOMB_LABEL;
                }
                s_labels.box_count++;
            }
            else if((ch == '.') || (ch == 'G') || (ch == '+'))
            {
                if(s_labels.goal_count >= SOKOBAN_MAX_BOXES) return 0u;
                s_labels.goal_count++;
            }
        }
    }
    return 1u;
}

static void mission_read_frame(void)
{
    uint32_t sequence;
    uint8_t row;
    uint8_t col;
    uint8_t ignored_all_zero;

    s_frame_updated = 0u;
    if(!camera_sokoban_copy_latest_frame(s_current_map, &row, &col,
                                         &ignored_all_zero, &sequence)) return;
    if(sequence == s_last_frame_sequence) return;
    s_last_frame_sequence = sequence;
    s_frame_updated = 1u;
    s_player_row = row;
    s_player_col = col;
}

static uint8_t mission_start_segment(const sokoban_solution_t *source,
                                     uint16_t begin,
                                     uint16_t end)
{
    uint16_t idx;
    mission_solution_reset(&s_segment_solution);
    if((begin > end) || (end > source->move_count)) return 0u;
    for(idx = begin; idx < end; idx++)
    {
        s_segment_solution.move_seq[idx - begin] = source->move_seq[idx];
    }
    s_segment_solution.move_count = (uint16_t)(end - begin);
    s_segment_solution.move_seq[s_segment_solution.move_count] = '\0';
    s_segment_solution.solved = 1u;
    if(s_segment_solution.move_count == 0u) return 1u;
    return path_executor_start(&s_segment_solution);
}

static uint8_t mission_start_one_move(char move, float distance)
{
    mission_solution_reset(&s_segment_solution);
    s_segment_solution.solved = 1u;
    s_segment_solution.move_count = 1u;
    s_segment_solution.move_seq[0] = move;
    s_segment_solution.move_seq[1] = '\0';
    return path_executor_start_with_distance(&s_segment_solution, distance);
}

static int8_t mission_find_box_at(const mission_tracked_box_t *boxes,
                                  uint8_t row,
                                  uint8_t col)
{
    uint8_t idx;
    for(idx = 0u; idx < s_tracked_box_count; idx++)
    {
        if((boxes[idx].active != 0u) &&
           (boxes[idx].row == row) && (boxes[idx].col == col))
            return (int8_t)idx;
    }
    return -1;
}

static int8_t mission_find_goal_at(const mission_tracked_goal_t *goals,
                                   uint8_t row,
                                   uint8_t col)
{
    uint8_t idx;
    for(idx = 0u; idx < s_tracked_goal_count; idx++)
    {
        if((goals[idx].row == row) && (goals[idx].col == col))
            return (int8_t)idx;
    }
    return -1;
}

static uint8_t mission_move_delta(char move, int8_t *dr, int8_t *dc)
{
    *dr = 0;
    *dc = 0;
    if(move == 'U') *dr = -1;
    else if(move == 'D') *dr = 1;
    else if(move == 'L') *dc = -1;
    else if(move == 'R') *dc = 1;
    else return 0u;
    return 1u;
}

static uint8_t mission_prepare_next_push_segment(void)
{
    uint16_t idx;
    uint8_t player_row = s_expected_player_row;
    uint8_t player_col = s_expected_player_col;

    (void)memcpy(s_predicted_map, s_expected_map, sizeof(s_predicted_map));
    (void)memcpy(s_predicted_boxes, s_tracked_boxes,
                 sizeof(s_predicted_boxes));
    (void)memcpy(s_predicted_goals, s_tracked_goals,
                 sizeof(s_predicted_goals));
    (void)memset(&s_push_checkpoint, 0, sizeof(s_push_checkpoint));
    s_push_checkpoint.box_index = SOKOBAN_INVALID_CELL;
    s_push_checkpoint.goal_index = SOKOBAN_INVALID_CELL;

    for(idx = s_task_move_index; idx < s_task_solution.move_count; idx++)
    {
        int8_t dr;
        int8_t dc;
        int16_t next_row;
        int16_t next_col;
        char next_cell;
        if(!mission_move_delta(s_task_solution.move_seq[idx], &dr, &dc))
            return 0u;
        next_row = (int16_t)player_row + dr;
        next_col = (int16_t)player_col + dc;
        if((next_row < 0) || (next_col < 0) ||
           (next_row >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
           (next_col >= (int16_t)SOKOBAN_MAP_WIDTH)) return 0u;
        next_cell = s_predicted_map[next_row][next_col];
        if((next_cell == '$') || (next_cell == '*'))
        {
            int16_t target_row = next_row + dr;
            int16_t target_col = next_col + dc;
            char target_cell;
            int8_t box_index = -1;
            int8_t goal_index = -1;
            if((target_row <= 0) || (target_col <= 0) ||
               (target_row + 1 >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
               (target_col + 1 >= (int16_t)SOKOBAN_MAP_WIDTH)) return 0u;
            target_cell = s_predicted_map[target_row][target_col];
            if(s_current_level != 1u)
            {
                box_index = mission_find_box_at(s_predicted_boxes,
                                                (uint8_t)next_row,
                                                (uint8_t)next_col);
                if(box_index < 0) return 0u;
            }
            s_push_checkpoint.valid = 1u;
            s_push_checkpoint.box_index = (box_index < 0) ?
                                          SOKOBAN_INVALID_CELL :
                                          (uint8_t)box_index;
            s_push_checkpoint.is_bomb = (uint8_t)(next_cell == '*');
            s_push_checkpoint.source_row = (uint8_t)next_row;
            s_push_checkpoint.source_col = (uint8_t)next_col;
            s_push_checkpoint.target_row = (uint8_t)target_row;
            s_push_checkpoint.target_col = (uint8_t)target_col;
            s_predicted_map[next_row][next_col] = '-';
            player_row = (uint8_t)next_row;
            player_col = (uint8_t)next_col;

            if((next_cell == '*') && (target_cell == '#'))
            {
                uint8_t row;
                uint8_t col;
                s_push_checkpoint.blast = 1u;
                if(box_index >= 0)
                    s_predicted_boxes[(uint8_t)box_index].active = 0u;
                for(row = (uint8_t)(target_row - 1);
                    row <= (uint8_t)(target_row + 1); row++)
                {
                    for(col = (uint8_t)(target_col - 1);
                        col <= (uint8_t)(target_col + 1); col++)
                    {
                        if((row != 0u) && (col != 0u) &&
                           (row + 1u != SOKOBAN_MAP_HEIGHT) &&
                           (col + 1u != SOKOBAN_MAP_WIDTH) &&
                           (s_predicted_map[row][col] == '#'))
                            s_predicted_map[row][col] = '-';
                    }
                }
            }
            else
            {
                if((target_cell != '-') && (target_cell != '.')) return 0u;
                if((next_cell == '*') && (target_cell == '.')) return 0u;
                if((next_cell == '$') && (target_cell == '.'))
                {
                    if(s_current_level != 1u)
                    {
                        goal_index = mission_find_goal_at(
                            s_predicted_goals,
                            (uint8_t)target_row,
                            (uint8_t)target_col);
                        if((goal_index < 0) ||
                           (s_predicted_boxes[(uint8_t)box_index].label !=
                            s_predicted_goals[(uint8_t)goal_index].label))
                            return 0u;
                        s_predicted_boxes[(uint8_t)box_index].active = 0u;
                        s_predicted_goals[(uint8_t)goal_index].active = 0u;
                        s_push_checkpoint.goal_index = (uint8_t)goal_index;
                    }
                    s_predicted_map[target_row][target_col] = '-';
                }
                else
                {
                    s_predicted_map[target_row][target_col] = next_cell;
                    if(box_index >= 0)
                    {
                        s_predicted_boxes[(uint8_t)box_index].row =
                            (uint8_t)target_row;
                        s_predicted_boxes[(uint8_t)box_index].col =
                            (uint8_t)target_col;
                    }
                }
            }
            s_predicted_player_row = player_row;
            s_predicted_player_col = player_col;
            s_push_checkpoint.player_row = player_row;
            s_push_checkpoint.player_col = player_col;
            if(!mission_start_segment(&s_task_solution, s_task_move_index,
                                      (uint16_t)(idx + 1u))) return 0u;
            s_task_move_index = (uint16_t)(idx + 1u);
            s_segment_running = 1u;
            mission_set_state(MISSION_EXECUTE_PUSH);
            return 1u;
        }
        if((next_cell != '-') && (next_cell != '.')) return 0u;
        player_row = (uint8_t)next_row;
        player_col = (uint8_t)next_col;
    }
    return 0u;
}

static void mission_reset_push_map_acquisition(void)
{
    s_map_stable_frames = 0u;
    s_frame_updated = 0u;
    s_last_frame_sequence = camera_frame_sequence;
    (void)memset(s_candidate_map, 0, sizeof(s_candidate_map));
}

static void mission_start_return(void)
{
    s_segment_running = 0u;
    if(s_have_valid_map != 0u)
    {
        mission_set_state((s_level_result == MISSION_LEVEL_RESULT_ABORTED) ?
                          MISSION_PLAN_ABORT_RETURN_ENTRY :
                          MISSION_PLAN_NORMAL_RETURN_ENTRY);
    }
    else
    {
        mission_set_state((s_level_result == MISSION_LEVEL_RESULT_ABORTED) ?
                          MISSION_ABORT_RETURN_PHYSICAL_START :
                          MISSION_NORMAL_RETURN_PHYSICAL_START);
    }
}

static void mission_abort_level(sokoban_status_t status)
{
    if((s_state == MISSION_FAULT) || (s_state == MISSION_FINISHED) ||
       (s_level_result == MISSION_LEVEL_RESULT_ABORTED)) return;
    s_last_status = status;
    s_level_result = MISSION_LEVEL_RESULT_ABORTED;
    path_executor_abort();
    MecanumCarStop();
    mission_start_return();
}

static void mission_commit_predicted_state(void)
{
    (void)memcpy(s_expected_map, s_predicted_map, sizeof(s_expected_map));
    (void)memcpy(s_tracked_boxes, s_predicted_boxes,
                 sizeof(s_tracked_boxes));
    (void)memcpy(s_tracked_goals, s_predicted_goals,
                 sizeof(s_tracked_goals));
    s_expected_player_row = s_predicted_player_row;
    s_expected_player_col = s_predicted_player_col;
}

static int8_t mission_find_rebuilt_box_at(
    const mission_tracked_box_t *boxes,
    uint8_t row,
    uint8_t col)
{
    uint8_t idx;
    for(idx = 0u; idx < s_tracked_box_count; idx++)
    {
        if((boxes[idx].active != 0u) &&
           (boxes[idx].row == row) && (boxes[idx].col == col))
            return (int8_t)idx;
    }
    return -1;
}

static uint8_t mission_rebuild_labeled_state(
    const char actual_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t player_row,
    uint8_t player_col,
    mission_recovery_status_t *recovery_status)
{
    mission_tracked_box_t rebuilt_boxes[SOKOBAN_MAX_BOXES];
    mission_tracked_goal_t rebuilt_goals[SOKOBAN_MAX_BOXES];
    uint8_t actual_rows[SOKOBAN_MAX_BOXES];
    uint8_t actual_cols[SOKOBAN_MAX_BOXES];
    uint8_t actual_matched[SOKOBAN_MAX_BOXES];
    uint8_t actual_count = 0u;
    uint8_t player_changed;
    uint8_t box_changed = 0u;
    uint8_t idx;
    uint8_t row;
    uint8_t col;

    if(!mission_cells_near(player_row, player_col,
                           s_predicted_player_row,
                           s_predicted_player_col)) return 0u;
    player_changed = (uint8_t)((player_row != s_predicted_player_row) ||
                               (player_col != s_predicted_player_col));
    (void)memcpy(rebuilt_boxes, s_predicted_boxes, sizeof(rebuilt_boxes));
    (void)memcpy(rebuilt_goals, s_predicted_goals, sizeof(rebuilt_goals));
    (void)memset(actual_matched, 0, sizeof(actual_matched));

    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(actual_map[row][col] == '$')
            {
                if(actual_count >= SOKOBAN_MAX_BOXES) return 0u;
                actual_rows[actual_count] = row;
                actual_cols[actual_count] = col;
                actual_count++;
            }
        }
    }

    for(idx = 0u; idx < s_tracked_box_count; idx++)
    {
        uint8_t actual_idx;
        uint8_t found = 0u;
        if((rebuilt_boxes[idx].bomb != 0u) ||
           (idx == s_push_checkpoint.box_index) ||
           (rebuilt_boxes[idx].active == 0u)) continue;
        for(actual_idx = 0u; actual_idx < actual_count; actual_idx++)
        {
            if((actual_matched[actual_idx] == 0u) &&
               (actual_rows[actual_idx] == rebuilt_boxes[idx].row) &&
               (actual_cols[actual_idx] == rebuilt_boxes[idx].col))
            {
                actual_matched[actual_idx] = 1u;
                found = 1u;
                break;
            }
        }
        if(found == 0u) return 0u;
    }

    if((s_push_checkpoint.is_bomb == 0u) &&
       (s_push_checkpoint.box_index < s_tracked_box_count))
    {
        uint8_t box_index = s_push_checkpoint.box_index;
        uint8_t candidate = SOKOBAN_INVALID_CELL;
        uint8_t candidate_count = 0u;
        uint8_t same_offset_candidate = SOKOBAN_INVALID_CELL;
        int8_t player_offset_row = (int8_t)player_row -
                                   (int8_t)s_predicted_player_row;
        int8_t player_offset_col = (int8_t)player_col -
                                   (int8_t)s_predicted_player_col;
        uint8_t expected_row = (rebuilt_boxes[box_index].active != 0u) ?
                               rebuilt_boxes[box_index].row :
                               s_push_checkpoint.target_row;
        uint8_t expected_col = (rebuilt_boxes[box_index].active != 0u) ?
                               rebuilt_boxes[box_index].col :
                               s_push_checkpoint.target_col;
        uint8_t actual_idx;
        for(actual_idx = 0u; actual_idx < actual_count; actual_idx++)
        {
            if((actual_matched[actual_idx] == 0u) &&
               mission_cells_near(actual_rows[actual_idx],
                                  actual_cols[actual_idx],
                                  expected_row, expected_col))
            {
                candidate = actual_idx;
                candidate_count++;
                if(((int16_t)actual_rows[actual_idx] ==
                    (int16_t)expected_row + player_offset_row) &&
                   ((int16_t)actual_cols[actual_idx] ==
                    (int16_t)expected_col + player_offset_col))
                {
                    same_offset_candidate = actual_idx;
                }
            }
        }
        if(same_offset_candidate != SOKOBAN_INVALID_CELL)
        {
            candidate = same_offset_candidate;
            candidate_count = 1u;
        }
        else if(candidate_count > 1u)
            return 0u;
        if(candidate_count == 1u)
        {
            rebuilt_boxes[box_index].active = 1u;
            rebuilt_boxes[box_index].row = actual_rows[candidate];
            rebuilt_boxes[box_index].col = actual_cols[candidate];
            actual_matched[candidate] = 1u;
            box_changed = (uint8_t)(
                (actual_rows[candidate] != expected_row) ||
                (actual_cols[candidate] != expected_col) ||
                (s_predicted_boxes[box_index].active == 0u));
            if(box_changed != 0u)
            {
                if((player_row == actual_rows[candidate]) &&
                   (player_col == actual_cols[candidate])) return 0u;
                if(!mission_cells_near(player_row, player_col,
                                       actual_rows[candidate],
                                       actual_cols[candidate])) return 0u;
            }
        }
        else if(rebuilt_boxes[box_index].active != 0u)
        {
            return 0u;
        }
    }

    for(idx = 0u; idx < actual_count; idx++)
        if(actual_matched[idx] == 0u) return 0u;

    for(idx = 0u; idx < s_tracked_goal_count; idx++)
        rebuilt_goals[idx].active = 0u;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(actual_map[row][col] == '.')
            {
                int8_t goal_index = mission_find_goal_at(
                    s_tracked_goals, row, col);
                if(goal_index < 0) return 0u;
                rebuilt_goals[(uint8_t)goal_index].active = 1u;
            }
        }
    }

    for(idx = 0u; idx < s_tracked_box_count; idx++)
    {
        uint8_t goal_idx;
        uint8_t matching_goal_active = 0u;
        if(rebuilt_boxes[idx].bomb != 0u)
        {
            rebuilt_boxes[idx].active = 0u;
            continue;
        }
        for(goal_idx = 0u; goal_idx < s_tracked_goal_count; goal_idx++)
        {
            if(rebuilt_goals[goal_idx].label == rebuilt_boxes[idx].label)
            {
                matching_goal_active = rebuilt_goals[goal_idx].active;
                break;
            }
        }
        if(goal_idx >= s_tracked_goal_count) return 0u;
        if(rebuilt_boxes[idx].active != matching_goal_active) return 0u;
    }

    {
        uint8_t bomb_slot = 0u;
        for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
        {
            for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
            {
                if(actual_map[row][col] == '*')
                {
                    while((bomb_slot < s_tracked_box_count) &&
                          (s_tracked_boxes[bomb_slot].bomb == 0u)) bomb_slot++;
                    if(bomb_slot >= s_tracked_box_count) return 0u;
                    rebuilt_boxes[bomb_slot].active = 1u;
                    rebuilt_boxes[bomb_slot].row = row;
                    rebuilt_boxes[bomb_slot].col = col;
                    rebuilt_boxes[bomb_slot].label = SOKOBAN_BOMB_LABEL;
                    bomb_slot++;
                }
            }
        }
    }

    (void)memset(&s_labels, 0, sizeof(s_labels));
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = actual_map[row][col];
            if((ch == '$') || (ch == '*'))
            {
                int8_t box_index;
                if(s_labels.box_count >= SOKOBAN_MAX_BOXES) return 0u;
                if(ch == '*')
                {
                    s_labels.box_labels[s_labels.box_count] =
                        SOKOBAN_BOMB_LABEL;
                }
                else
                {
                    box_index = mission_find_rebuilt_box_at(
                        rebuilt_boxes, row, col);
                    if((box_index < 0) ||
                       (rebuilt_boxes[(uint8_t)box_index].bomb != 0u)) return 0u;
                    s_labels.box_labels[s_labels.box_count] =
                        rebuilt_boxes[(uint8_t)box_index].label;
                }
                s_labels.box_count++;
            }
            else if(ch == '.')
            {
                int8_t goal_index;
                if(s_labels.goal_count >= SOKOBAN_MAX_BOXES) return 0u;
                goal_index = mission_find_goal_at(rebuilt_goals, row, col);
                if((goal_index < 0) ||
                   (rebuilt_goals[(uint8_t)goal_index].active == 0u)) return 0u;
                s_labels.goal_labels[s_labels.goal_count++] =
                    rebuilt_goals[(uint8_t)goal_index].label;
            }
        }
    }

    (void)memcpy(s_tracked_boxes, rebuilt_boxes, sizeof(s_tracked_boxes));
    (void)memcpy(s_tracked_goals, rebuilt_goals, sizeof(s_tracked_goals));
    (void)memcpy(s_expected_map, actual_map, sizeof(s_expected_map));
    s_expected_player_row = player_row;
    s_expected_player_col = player_col;
    if(box_changed != 0u) *recovery_status = MISSION_RECOVERY_BOX;
    else if(player_changed != 0u) *recovery_status = MISSION_RECOVERY_PLAYER;
    else *recovery_status = MISSION_RECOVERY_ACTUAL_MAP;
    return 1u;
}

static void mission_finish_success(void)
{
    s_level_result = MISSION_LEVEL_RESULT_SUCCESS;
    mission_start_return();
}

static void mission_handle_push_checkpoint(void)
{
    mission_recovery_status_t recovery_status = MISSION_RECOVERY_ACTUAL_MAP;
    uint8_t exact_map;
    uint8_t exact_player;
    uint8_t recovered = 0u;

    mission_normalize_map(s_current_map, s_solver_map);
    exact_map = (uint8_t)(memcmp(s_solver_map, s_predicted_map,
                                 sizeof(s_solver_map)) == 0);
    exact_player = (uint8_t)((s_player_row == s_predicted_player_row) &&
                             (s_player_col == s_predicted_player_col));
    s_push_checkpoint_index++;
    if(s_push_checkpoint.blast != 0u) s_blast_event_index++;

    if((exact_map != 0u) && (exact_player != 0u))
    {
        mission_commit_predicted_state();
        s_recovery_status = MISSION_RECOVERY_EXACT;
    }
    else
    {
        if(s_recovery_count >= MISSION_MAX_RECOVERIES)
        {
            s_recovery_status = MISSION_RECOVERY_REJECTED;
            mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            return;
        }
        if(!mission_cells_near(s_player_row, s_player_col,
                               s_predicted_player_row,
                               s_predicted_player_col))
        {
            s_recovery_status = MISSION_RECOVERY_REJECTED;
            mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            return;
        }
        if(s_current_level == 1u)
        {
            (void)memcpy(s_expected_map, s_solver_map,
                         sizeof(s_expected_map));
            s_expected_player_row = s_player_row;
            s_expected_player_col = s_player_col;
            recovery_status = (exact_map != 0u) ?
                              MISSION_RECOVERY_PLAYER :
                              MISSION_RECOVERY_ACTUAL_MAP;
            recovered = 1u;
        }
        else if(mission_rebuild_labeled_state(
                    s_solver_map, s_player_row, s_player_col,
                    &recovery_status))
        {
            recovered = 1u;
        }
        if(recovered == 0u)
        {
            s_recovery_status = MISSION_RECOVERY_REJECTED;
            mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            return;
        }
        s_recovery_count++;
        s_recovery_status = recovery_status;
    }

    if(mission_task_complete(s_current_map))
    {
        mission_finish_success();
        return;
    }
    if(recovered != 0u)
    {
        mission_set_state((s_current_level == 1u) ?
                          MISSION_PLAN_PLAIN_PUSH :
                          MISSION_PLAN_LABELED_PUSH);
    }
    else if(!mission_prepare_next_push_segment())
    {
        mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
    }
}

static void mission_update_push_map_stability(void)
{
    uint8_t same;
    if(s_frame_updated == 0u) return;
    if(!mission_checkpoint_map_valid(s_current_map,
                                     s_player_row, s_player_col))
    {
        s_map_stable_frames = 0u;
        return;
    }
    same = (uint8_t)((s_map_stable_frames != 0u) &&
                     (s_candidate_player_row == s_player_row) &&
                     (s_candidate_player_col == s_player_col) &&
                     (memcmp(s_candidate_map, s_current_map,
                             sizeof(s_candidate_map)) == 0));
    if(same != 0u)
    {
        if(s_map_stable_frames < MISSION_MAP_STABLE_FRAMES)
            s_map_stable_frames++;
    }
    else
    {
        (void)memcpy(s_candidate_map, s_current_map,
                     sizeof(s_candidate_map));
        s_candidate_player_row = s_player_row;
        s_candidate_player_col = s_player_col;
        s_map_stable_frames = 1u;
    }
    if(s_map_stable_frames >= MISSION_MAP_STABLE_FRAMES)
        mission_handle_push_checkpoint();
}

static void mission_advance_level(void)
{
    s_last_level_result = s_level_result;
    if(s_current_level >= MISSION_LAST_LEVEL)
    {
        s_timer_running = 0u;
        MecanumCarStop();
        mission_set_state(MISSION_FINISHED);
        return;
    }
    s_current_level++;
    mission_reset_level_data();
    mission_set_state(MISSION_LEVEL_CHANGE);
}

static void mission_begin_inspection(void)
{
    s_last_status = sokoban_plan_inspection(s_initial_map, s_heading,
                                            &s_inspection_plan);
    if(s_last_status != SOKOBAN_STATUS_OK)
    {
        mission_abort_level(s_last_status);
        return;
    }
    if(!mission_initialize_labels())
    {
        mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
        return;
    }
    s_inspection_event = 0u;
    s_inspection_move_index = 0u;
    s_segment_running = 0u;
    mission_set_state(MISSION_EXECUTE_INSPECTION);
}

static void mission_accept_stable_map(void)
{
    s_have_valid_map = 1u;
    s_entry_row = s_player_row;
    s_entry_col = s_player_col;
    (void)memcpy(s_initial_map, s_current_map, sizeof(s_initial_map));
    if(s_current_level == 1u)
    {
        mission_set_state(MISSION_PLAN_PLAIN_PUSH);
    }
    else
    {
        mission_set_state(MISSION_PLAN_INSPECTION);
    }
}

static void mission_update_map_stability(void)
{
    uint8_t same;
    if(s_frame_updated == 0u) return;
    if(!mission_map_structurally_valid(s_current_map, s_player_row,
                                       s_player_col))
    {
        s_map_stable_frames = 0u;
        return;
    }

    same = (uint8_t)((s_map_stable_frames != 0u) &&
                     (s_candidate_player_row == s_player_row) &&
                     (s_candidate_player_col == s_player_col) &&
                     (memcmp(s_candidate_map, s_current_map,
                             sizeof(s_candidate_map)) == 0));
    if(same != 0u)
    {
        if(s_map_stable_frames < MISSION_MAP_STABLE_FRAMES)
        {
            s_map_stable_frames++;
        }
    }
    else
    {
        (void)memcpy(s_candidate_map, s_current_map,
                     sizeof(s_candidate_map));
        s_candidate_player_row = s_player_row;
        s_candidate_player_col = s_player_col;
        s_map_stable_frames = 1u;
    }

    if(s_map_stable_frames >= MISSION_MAP_STABLE_FRAMES)
    {
        mission_accept_stable_map();
    }
}

void mission_controller_init(void)
{
    s_state = MISSION_BOOT_DELAY;
    s_timer_running = 0u;
    s_elapsed_10ms = 0u;
    s_state_elapsed_10ms = 0u;
    s_abort_hold_10ms = 0u;
    s_current_level = MISSION_FIRST_LEVEL;
    s_level_result = MISSION_LEVEL_RESULT_NONE;
    s_last_level_result = MISSION_LEVEL_RESULT_NONE;
    s_fatal_fault = MISSION_FATAL_NONE;
    s_heading = SOKOBAN_DIR_UP;
    s_entry_distance = 24.0f;
    s_last_status = SOKOBAN_STATUS_OK;
    mission_reset_level_data();
    path_executor_set_heading(s_heading);
    MecanumCarStop();
}

void mission_controller_update_10ms(void)
{
    s_state_elapsed_10ms++;
    if(s_timer_running != 0u) s_elapsed_10ms++;
    if(s_state == MISSION_WAIT_PUSH_SETTLE)
    {
        if(mission_encoder_is_still() != 0u)
        {
            if(s_push_settle_10ms < MISSION_PUSH_SETTLE_STILL_10MS)
                s_push_settle_10ms++;
        }
        else
        {
            s_push_settle_10ms = 0u;
        }
    }
    if(s_state == MISSION_ABORT_HOLD)
    {
        if(mission_encoder_is_still() != 0u)
        {
            if(s_abort_hold_10ms < MISSION_ABORT_HOLD_10MS)
            {
                s_abort_hold_10ms++;
            }
        }
        else
        {
            s_abort_hold_10ms = 0u;
        }
    }
}

void mission_controller_process(void)
{
    mission_read_frame();
    if((s_state == MISSION_FAULT) || (s_state == MISSION_FINISHED))
    {
        MecanumCarStop();
        return;
    }
    if(path_executor_is_fault())
    {
        mission_enter_fatal(MISSION_FATAL_PATH, SOKOBAN_STATUS_INVALID_MAP);
        return;
    }

    switch(s_state)
    {
        case MISSION_BOOT_DELAY:
            MecanumCarStop();
            if(s_state_elapsed_10ms >= MISSION_BOOT_DELAY_10MS)
            {
                mission_set_state(MISSION_LEVEL_CHANGE);
            }
            break;

        case MISSION_LEVEL_CHANGE:
            mission_reset_map_acquisition();
            if(mission_start_one_move('R', 24.0f))
            {
                s_timer_running = 1u;
                mission_set_state(MISSION_LEAVE_START_ZONE);
            }
            else
            {
                mission_enter_fatal(MISSION_FATAL_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_LEAVE_START_ZONE:
            if(path_executor_is_done())
            {
                s_entry_distance = path_executor_get_last_step_distance();
                mission_reset_map_acquisition();
                mission_set_state(MISSION_WAIT_VALID_MAP);
            }
            break;

        case MISSION_WAIT_VALID_MAP:
            mission_update_map_stability();
            if((s_state == MISSION_WAIT_VALID_MAP) &&
               (s_state_elapsed_10ms >= MISSION_MAP_WAIT_TIMEOUT_10MS))
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_PLAN_PLAIN_PUSH:
            if((s_tracking_initialized == 0u) &&
               !mission_initialize_tracking(s_entry_row, s_entry_col))
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
                break;
            }
            mission_build_solver_map();
            mission_solution_reset(&s_task_solution);
            s_last_status = sokoban_solve_decomposed(s_solver_map,
                                                      &s_task_solution);
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                s_last_status = sokoban_solve_bidirectional_astar(
                    s_solver_map, &s_task_solution);
            }
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                mission_abort_level(s_last_status);
            }
            else
            {
                s_task_move_index = 0u;
                s_push_checkpoint_total = (uint16_t)(
                    s_push_checkpoint_index + s_task_solution.push_count);
                if(!mission_prepare_next_push_segment())
                    mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_PLAN_INSPECTION:
            mission_begin_inspection();
            break;

        case MISSION_EXECUTE_INSPECTION:
            if(s_inspection_event >= s_inspection_plan.event_count)
            {
                mission_set_state(MISSION_PLAN_LABELED_PUSH);
                break;
            }
            if(s_segment_running == 0u)
            {
                uint16_t end =
                    s_inspection_plan.events[s_inspection_event].move_index;
                if(end == s_inspection_move_index)
                {
                    mission_set_state(MISSION_WAIT_TURN);
                }
                else if(mission_start_segment(&s_inspection_plan.route,
                                              s_inspection_move_index, end))
                {
                    s_segment_running = 1u;
                }
                else
                {
                    mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(path_executor_is_done())
            {
                s_segment_running = 0u;
                s_inspection_move_index =
                    s_inspection_plan.events[s_inspection_event].move_index;
                mission_set_state(MISSION_WAIT_TURN);
            }
            break;

        case MISSION_WAIT_TURN:
        {
            const sokoban_inspection_event_t *event =
                &s_inspection_plan.events[s_inspection_event];
            if((event->quarter_turns == 0) || (s_turn_complete != 0u))
            {
                s_turn_complete = 0u;
                s_heading = event->face_direction;
                path_executor_set_heading(s_heading);
                mission_set_state(MISSION_WAIT_SCAN_RESULT);
            }
            else if(s_state_elapsed_10ms >= MISSION_TURN_TIMEOUT_10MS)
            {
                mission_enter_fatal(MISSION_FATAL_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
        } break;

        case MISSION_WAIT_SCAN_RESULT:
        {
            const sokoban_inspection_event_t *event =
                &s_inspection_plan.events[s_inspection_event];
            label = (event->object_type == (uint8_t)SOKOBAN_OBJECT_BOX) ?
                    s_labels.box_labels[event->object_index] :
                    s_labels.goal_labels[event->object_index];
            if(label != 0u)
            {
                if((uint8_t)(s_inspection_event + 2u) ==
                   s_inspection_plan.event_count)
                {
                    s_inspection_plan.final_row = event->stand_row;
                    s_inspection_plan.final_col = event->stand_col;
                    s_inspection_plan.final_heading = event->face_direction;
                    s_inspection_event = s_inspection_plan.event_count;
                }
                else
                {
                    s_inspection_event++;
                }
                mission_set_state(MISSION_EXECUTE_INSPECTION);
            }
        } break;

        case MISSION_PLAN_LABELED_PUSH:
            if((s_tracking_initialized == 0u) &&
               !mission_initialize_tracking(
                   s_inspection_plan.final_row,
                   s_inspection_plan.final_col))
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_LABELS);
                break;
            }
            mission_build_solver_map();
            s_last_status = sokoban_solve_labeled(
                s_solver_map,
                s_expected_player_row,
                s_expected_player_col,
                &s_labels,
                &s_task_solution);
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                mission_abort_level(s_last_status);
            }
            else
            {
                s_task_move_index = 0u;
                s_push_checkpoint_total = (uint16_t)(
                    s_push_checkpoint_index + s_task_solution.push_count);
                s_blast_total = (uint8_t)(
                    s_blast_event_index + s_task_solution.blast_count);
                if(!mission_prepare_next_push_segment())
                    mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_EXECUTE_PUSH:
            if(path_executor_is_done())
            {
                s_segment_running = 0u;
                MecanumCarStop();
                s_push_settle_10ms = 0u;
                mission_set_state(MISSION_WAIT_PUSH_SETTLE);
            }
            break;

        case MISSION_WAIT_PUSH_SETTLE:
            MecanumCarStop();
            if(s_push_settle_10ms >= MISSION_PUSH_SETTLE_STILL_10MS)
            {
                mission_reset_push_map_acquisition();
                mission_set_state(MISSION_WAIT_PUSH_MAP);
            }
            else if(s_state_elapsed_10ms >= MISSION_PUSH_SETTLE_TIMEOUT_10MS)
            {
                mission_enter_fatal(MISSION_FATAL_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_WAIT_PUSH_MAP:
            MecanumCarStop();
            mission_update_push_map_stability();
            if((s_state == MISSION_WAIT_PUSH_MAP) &&
               (s_state_elapsed_10ms >= MISSION_RENDER_TIMEOUT_10MS))
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            }
            break;

        case MISSION_PLAN_NORMAL_RETURN_ENTRY:
        case MISSION_PLAN_ABORT_RETURN_ENTRY:
        {
            sokoban_status_t return_status = sokoban_plan_walk(
                s_current_map, s_player_row, s_player_col,
                s_entry_row, s_entry_col, &s_task_solution);
            if(return_status != SOKOBAN_STATUS_OK)
            {
                mission_enter_fatal(MISSION_FATAL_RETURN_PATH, return_status);
            }
            else if(s_task_solution.move_count == 0u)
            {
                mission_set_state((s_level_result == MISSION_LEVEL_RESULT_ABORTED) ?
                                  MISSION_ABORT_RETURN_PHYSICAL_START :
                                  MISSION_NORMAL_RETURN_PHYSICAL_START);
            }
            else if(path_executor_start(&s_task_solution))
            {
                mission_set_state((s_level_result == MISSION_LEVEL_RESULT_ABORTED) ?
                                  MISSION_EXECUTE_ABORT_RETURN_ENTRY :
                                  MISSION_EXECUTE_NORMAL_RETURN_ENTRY);
            }
            else
            {
                mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
        } break;

        case MISSION_EXECUTE_NORMAL_RETURN_ENTRY:
        case MISSION_EXECUTE_ABORT_RETURN_ENTRY:
            if(path_executor_is_done())
            {
                s_segment_running = 0u;
                mission_set_state((s_level_result == MISSION_LEVEL_RESULT_ABORTED) ?
                                  MISSION_ABORT_RETURN_PHYSICAL_START :
                                  MISSION_NORMAL_RETURN_PHYSICAL_START);
            }
            break;

        case MISSION_NORMAL_RETURN_PHYSICAL_START:
        case MISSION_ABORT_RETURN_PHYSICAL_START:
            if(s_segment_running == 0u)
            {
                if(mission_start_one_move('L', s_entry_distance))
                {
                    s_segment_running = 1u;
                }
                else
                {
                    mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(path_executor_is_done())
            {
                s_segment_running = 0u;
                MecanumCarStop();
                if(s_level_result == MISSION_LEVEL_RESULT_ABORTED)
                {
                    s_abort_hold_10ms = 0u;
                    mission_set_state(MISSION_ABORT_HOLD);
                }
                else
                {
                    mission_advance_level();
                }
            }
            break;

        case MISSION_ABORT_HOLD:
            MecanumCarStop();
            if(s_abort_hold_10ms >= MISSION_ABORT_HOLD_10MS)
            {
                mission_advance_level();
            }
            break;

        case MISSION_FINISHED:
        case MISSION_FAULT:
        default:
            MecanumCarStop();
            break;
    }
}

void mission_controller_complete_turn(void)
{
    s_turn_complete = 1u;
}

void mission_controller_emergency_stop(void)
{
    if((s_state != MISSION_FAULT) && (s_state != MISSION_FINISHED))
    {
        mission_enter_fatal(MISSION_FATAL_EMERGENCY_STOP,
                            SOKOBAN_STATUS_INVALID_MAP);
    }
}

uint8_t mission_controller_set_label(sokoban_object_type_t type,
                                     uint8_t object_index,
                                     uint8_t value)
{
    if(value == 0u) return 0u;
    if((type == SOKOBAN_OBJECT_BOX) && (object_index < s_labels.box_count))
    {
        s_labels.box_labels[object_index] = value;
        return 1u;
    }
    if((type == SOKOBAN_OBJECT_GOAL) &&
       (object_index < s_labels.goal_count))
    {
        s_labels.goal_labels[object_index] = value;
        return 1u;
    }
    return 0u;
}

mission_state_t mission_controller_get_state(void)
{
    return s_state;
}

uint32_t mission_controller_get_elapsed_10ms(void)
{
    return s_elapsed_10ms;
}

sokoban_status_t mission_controller_get_last_status(void)
{
    return s_last_status;
}

uint8_t mission_controller_get_level(void)
{
    return s_current_level;
}

mission_level_result_t mission_controller_get_last_level_result(void)
{
    return s_last_level_result;
}

mission_level_result_t mission_controller_get_level_result(void)
{
    return s_level_result;
}

mission_fatal_fault_t mission_controller_get_fatal_fault(void)
{
    return s_fatal_fault;
}

uint16_t mission_controller_get_abort_hold_remaining_10ms(void)
{
    if(s_state != MISSION_ABORT_HOLD) return 0u;
    if(s_abort_hold_10ms >= MISSION_ABORT_HOLD_10MS) return 0u;
    return (uint16_t)(MISSION_ABORT_HOLD_10MS - s_abort_hold_10ms);
}

uint8_t mission_controller_get_map_stable_frames(void)
{
    return s_map_stable_frames;
}

uint16_t mission_controller_get_push_checkpoint_index(void)
{
    return s_push_checkpoint_index;
}

uint16_t mission_controller_get_push_checkpoint_total(void)
{
    return s_push_checkpoint_total;
}

uint8_t mission_controller_get_recovery_count(void)
{
    return s_recovery_count;
}

mission_recovery_status_t mission_controller_get_recovery_status(void)
{
    return s_recovery_status;
}

uint8_t mission_controller_should_stop(void)
{
    return (uint8_t)((s_state == MISSION_BOOT_DELAY) ||
                     (s_state == MISSION_WAIT_PUSH_SETTLE) ||
                     (s_state == MISSION_WAIT_PUSH_MAP) ||
                     (s_state == MISSION_ABORT_HOLD) ||
                     (s_state == MISSION_FINISHED) ||
                     (s_state == MISSION_FAULT));
}

const sokoban_inspection_event_t *mission_controller_get_pending_inspection(void)
{
    if(((s_state == MISSION_WAIT_TURN) ||
        (s_state == MISSION_WAIT_SCAN_RESULT)) &&
       (s_inspection_event < s_inspection_plan.event_count))
    {
        return &s_inspection_plan.events[s_inspection_event];
    }
    return 0;
}
static uint8_t turn_running;
static uint8_t stable_count;
static float turn_target_yaw;


static float wrap_angle(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while(angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

void mission_turn_process(void)         //转向模块
{
    const sokoban_inspection_event_t *event;
    float error;

    if(mission_controller_get_state() != MISSION_WAIT_TURN)
    {
        turn_running = 0u;
        stable_count = 0u;
        return;
    }

    event = mission_controller_get_pending_inspection();
    if(event == 0)
    {
        return;
    }

    if(event->quarter_turns == 0)
    {
        return; /* mission 自己会直接通过 */
    }

    if(turn_running == 0u)
    {
        target_vx = 0.0f;
        target_vy = 0.0f;

        turn_target_yaw =
            wrap_angle(target_yaw +
                       TURN_YAW_SIGN * event->quarter_turns * 90.0f);

        target_yaw = turn_target_yaw;
        turn_running = 1u;
        stable_count = 0u;
    }

    error = wrap_angle(turn_target_yaw - yaw);

    if((fabsf(error) <= 2.0f) && (fabsf(Gyro.x) <= 3.0f))
    {
        if(++stable_count >= 10u)
        {
            turn_running = 0u;
            stable_count = 0u;
            mission_controller_complete_turn();
        }
    }
    else
    {
        stable_count = 0u;
    }
}

static void label_uart_send_request(
    const sokoban_inspection_event_t *event)
{
    uint8_t frame[8];

    frame[0] = 0xA5u;
    frame[1] = 0xB5u;
    frame[2] = event->object_type;
    frame[3] = event->object_index;
    frame[4] = event->object_row;
    frame[5] = event->object_col;
    frame[6] = (uint8_t)(
        frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
    frame[7] = 0xC5u;

    label_uart_write_request(frame, sizeof(frame));
}

// void mission_label_process(void)
// {
//     static uint8_t request_sent;
//     const sokoban_inspection_event_t *event;
//     uint8_t type;
//     uint8_t index;
//     uint8_t label;

//     if(mission_controller_get_state() !=
//        MISSION_WAIT_SCAN_RESULT)
//     {
//         request_sent = 0u;
//         return;
//     }
//     else if(request_sent==0u)
//     {
//         event = mission_controller_get_pending_inspection();
//         if(event!=0)
//         {
//             label_uart_send_request(event);
//             request_sent = 1u;
//         }
//     }
//     if (label_rx_ready == 0u)
//     {
//         return;
//     }

//     __disable_irq();

//     type = label_rx_type;
//     index = label_rx_index;
//     label = label_rx_value;
//     label_rx_ready = 0u;

//     __enable_irq();

//     /*
//      * 防止上一物体的延迟结果错误地写入
//      * 当前物体。
//      */
//     if((type != event->object_type) ||
//        (index != event->object_index))
//     {
//         return;
//     }

//     if(label == 0u)
//     {
//         return;
//     }

//     if((type == (uint8_t)SOKOBAN_OBJECT_GOAL) &&
//        (label == SOKOBAN_BOMB_LABEL))
//     {
//         return;
//     }

//     (void)mission_controller_set_label(
//         (sokoban_object_type_t)type,
//         index,
//         label);
// }


void mission_label_process(void)
{
    static uint8_t request_sent = 0u;
    static uint8_t attempt_count = 0u;
    static uint8_t digit_backoff_used = 0u;
    static uint8_t digit_backoff_running = 0u;
    static uint32_t last_send_10ms = 0u;

    const sokoban_inspection_event_t *event;
    uint32_t now_10ms;

    uint8_t type;
    uint8_t index;
    uint8_t label;

    /*
     * 离开标签识别状态后，清除本次识别的
     * 发送和重试状态。
     */
    if(mission_controller_get_state() !=
       MISSION_WAIT_SCAN_RESULT)
    {
        request_sent = 0u;
        attempt_count = 0u;
        digit_backoff_used = 0u;
        digit_backoff_running = 0u;
        last_send_10ms = 0u;
        return;
    }

    /*
     * event每次调用都必须重新获取，
     * 不能只在首次发送时赋值。
     */
    event = mission_controller_get_pending_inspection();
    if(event == 0)
    {
        return;
    }

    if(digit_backoff_running != 0u)
    {
        if(path_executor_is_fault())
        {
            mission_enter_fatal(MISSION_FATAL_PATH,
                                SOKOBAN_STATUS_INVALID_LABELS);
            return;
        }
        if(!path_executor_is_done()) return;

        MecanumCarStop();
        MecanumSpeedPidReset();
        __disable_irq();
        label_rx_ready = 0u;
        __enable_irq();

        digit_backoff_running = 0u;
        request_sent = 0u;
        attempt_count = 0u;
        last_send_10ms = mission_controller_get_elapsed_10ms();
        return;
    }

    now_10ms = mission_controller_get_elapsed_10ms();

    /* 首次发送识别请求 */
    if(request_sent == 0u)
    {
        label_uart_send_request(event);

        request_sent = 1u;
        attempt_count = 1u;
        last_send_10ms = now_10ms;
    }

    /*
     * 优先处理已经收到的标签。
     * 避免刚收到结果时又触发一次超时重发。
     */
    if(label_rx_ready != 0u)
    {
        __disable_irq();

        type = label_rx_type;
        index = label_rx_index;
        label = label_rx_value;
        label_rx_ready = 0u;

        __enable_irq();

        /*
         * 类型和编号不一致，说明是上一物体的
         * 延迟结果，丢弃后继续等待当前结果。
         */
        if((type == event->object_type) &&
           (index == event->object_index) &&
           (label != 0u) &&
           (label <= LABEL_MAX_VALUE))
        {
            if(mission_controller_set_label(
                   (sokoban_object_type_t)type,
                   index,
                   label) != 0u)
            {
                return;
            }
        }

        if((type == event->object_type) &&
           (index == event->object_index) &&
           (type == (uint8_t)SOKOBAN_OBJECT_GOAL) &&
           (label == 0u))
        {
            if(digit_backoff_used != 0u)
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_LABELS);
                return;
            }

            MecanumCarStop();
            if(!path_executor_start_body_step_with_distance_and_tolerance(
                   SOKOBAN_BODY_BACKWARD,
                   DIGIT_RETRY_BACKOFF_DISTANCE,
                   DIGIT_RETRY_BACKOFF_TOLERANCE))
            {
                if(path_executor_is_fault())
                {
                    mission_enter_fatal(MISSION_FATAL_PATH,
                                        SOKOBAN_STATUS_INVALID_LABELS);
                }
                else
                {
                    mission_abort_level(SOKOBAN_STATUS_INVALID_LABELS);
                }
                return;
            }

            digit_backoff_used = 1u;
            digit_backoff_running = 1u;
            request_sent = 0u;
            attempt_count = 0u;
            __disable_irq();
            label_rx_ready = 0u;
            __enable_irq();
            return;
        }
    }

    /* 检查距离上次发送是否已经超过500 ms */
    if((uint32_t)(now_10ms - last_send_10ms) >=
       LABEL_REPLY_TIMEOUT_10MS)
    {
        if(attempt_count < LABEL_MAX_ATTEMPTS)
        {
            label_uart_send_request(event);

            attempt_count++;
            last_send_10ms = now_10ms;
        }
        else
        {
            mission_abort_level(SOKOBAN_STATUS_INVALID_LABELS);
        }
    }
}


uint8_t mission_controller_get_blast_event_index(void)
{
    return s_blast_event_index;
}

uint8_t mission_controller_get_blast_count(void)
{
    return s_blast_total;
}
