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
#define DIGIT_RETRY_BACKOFF_MAX_COUNT       (2u)
#define MISSION_BOOT_DELAY_10MS             (300u)
#define MISSION_TURN_TIMEOUT_10MS            (500u)
#define MISSION_ABORT_HOLD_10MS              (300u)
#define MISSION_MAP_STABLE_FRAMES            (2u)
#define MISSION_STILL_ENCODER_LIMIT          (2)
#define VISION_POSE_SAMPLE_COUNT             (3u)
#define VISION_POSE_MIN_CONFIDENCE           (60u)
#define VISION_POSE_MAX_SPREAD_CELL          (0.12f)
#define VISION_POSE_MAX_GYRO_RATE            (3.0f)
#define VISION_POSE_TIMEOUT_10MS              (80u)
#define VISION_ALIGNMENT_TOTAL_TIMEOUT_10MS  (200u)
#define VISION_CORRECTION_DEADBAND_CELL      (0.08f)
#define VISION_CORRECTION_FINAL_CELL         (0.15f)
#define VISION_CORRECTION_MAX_ATTEMPTS       (2u)
#define VISION_REPLAN_ERROR_CELL             (0.60f)
#define VISION_CORRECTION_CELL_DISTANCE      (22.0f)
#define VISION_CORRECTION_MIN_DISTANCE       (3.0f)
#define VISION_CORRECTION_MAX_DISTANCE       (13.2f)
#define VISION_LONG_CORRECTION_MAX_ERROR_CELL (1.0f)
#define VISION_LONG_CORRECTION_MAX_DISTANCE  (22.0f)
#define VISION_CORRECTION_PATH_TOLERANCE     (0.8f)
#define VISION_CORRECTION_DEADZONE_RATIO     (0.85f)
#define VISION_CORRECTION_REVERSE_GUARD_CELL (0.20f)
#define MISSION_FIRST_ENTRY_DISTANCE         (44.0f)
#define MISSION_ENTRY_CELL_DISTANCE          (22.0f)
#define MISSION_ENTRY_MAP_GRACE_10MS         (80u)
#define MISSION_LONG_STRAIGHT_MIN_CELLS      (5u)
#define MISSION_RETURN_ROW                   (5u)
#define MISSION_RETURN_COL                   (1u)
#define MISSION_RETURN_DISTANCE             (22.0f)
#define MISSION_RETURN_ZONE_TIMEOUT_10MS     (100u)

typedef enum
{
    MISSION_PUSH_IDLE = 0,
    MISSION_PUSH_PREPARE,
    MISSION_PUSH_WAIT_WALK,
    MISSION_PUSH_WAIT_WALK_POSE,
    MISSION_PUSH_WAIT_POSE,
    MISSION_PUSH_WAIT_PUSH,
    MISSION_PUSH_WAIT_MAP,
    MISSION_PUSH_WAIT_RETRY_POSE
} mission_push_phase_t;

typedef enum
{
    MISSION_PUSH_UPDATE_RUNNING = 0,
    MISSION_PUSH_UPDATE_DONE,
    MISSION_PUSH_UPDATE_FAULT
} mission_push_update_result_t;

typedef enum
{
    MISSION_ALIGN_WAITING = 0,
    MISSION_ALIGN_DONE,
    MISSION_ALIGN_REPLAN,
    MISSION_ALIGN_FAULT
} mission_align_result_t;

typedef struct
{
    uint8_t active;
    uint8_t moving;
    uint8_t target_row;
    uint8_t target_col;
    uint8_t prefer_x;
    uint8_t correction_count;
    uint8_t skip_on_timeout;
    uint8_t long_straight;
    uint8_t last_correction_x_axis;
    int8_t last_correction_sign;
    uint16_t wait_10ms;
    volatile uint16_t elapsed_10ms;
} mission_alignment_t;

typedef struct
{
    uint8_t active;
    uint8_t row;
    uint8_t col;
    uint8_t label;
    uint8_t bomb;
} mission_tracked_box_t;

typedef struct
{
    uint8_t active;
    uint8_t row;
    uint8_t col;
    uint8_t label;
} mission_tracked_goal_t;

static volatile mission_state_t s_state = MISSION_BOOT_DELAY;
static volatile uint8_t s_timer_running = 0u;
static volatile uint32_t s_elapsed_10ms = 0u;
static volatile uint32_t s_state_elapsed_10ms = 0u;
static volatile uint16_t s_abort_hold_10ms = 0u;
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
static uint8_t s_clear_frame_count = 0u;
static uint8_t s_blast_event_index = 0u;
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
static mission_push_phase_t s_push_phase = MISSION_PUSH_IDLE;
static uint16_t s_push_cursor = 0u;
static uint16_t s_push_range_end = 0u;
static uint16_t s_push_segment_end = 0u;
static uint8_t s_push_expected_row = 0u;
static uint8_t s_push_expected_col = 0u;
static uint32_t s_pose_last_sequence = 0u;
static uint8_t s_pose_sample_count = 0u;
static float s_pose_samples_x[VISION_POSE_SAMPLE_COUNT];
static float s_pose_samples_y[VISION_POSE_SAMPLE_COUNT];
static mission_alignment_t s_alignment;
static uint8_t s_visual_deadzone_active = 0u;
static float s_visual_deadzone_saved = 0.0f;
static uint8_t s_entry_retry_used = 0u;
static uint8_t s_entry_alignment_done = 0u;
static uint32_t s_push_map_baseline_sequence = 0u;
static uint8_t s_push_map_stable_count = 0u;
static uint8_t s_push_source_row = 0u;
static uint8_t s_push_source_col = 0u;
static uint8_t s_push_target_row = 0u;
static uint8_t s_push_target_col = 0u;
static char s_push_move = '\0';
static char s_push_before_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_push_candidate_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static mission_tracked_box_t s_tracked_boxes[SOKOBAN_MAX_BOXES];
static mission_tracked_goal_t s_tracked_goals[SOKOBAN_MAX_BOXES];
static uint8_t s_tracked_box_count = 0u;
static uint8_t s_tracked_goal_count = 0u;

static char s_current_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_candidate_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static char s_initial_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE];
static sokoban_inspection_plan_t s_inspection_plan;
static sokoban_label_table_t s_labels;
static sokoban_solution_t s_task_solution;
static sokoban_solution_t s_segment_solution;

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

static void mission_restore_visual_deadzone(void)
{
    if(s_visual_deadzone_active != 0u)
    {
        MecanumSetSpeedDeadzoneMinRatio(s_visual_deadzone_saved);
        s_visual_deadzone_active = 0u;
    }
}

static void mission_push_reset(void)
{
    mission_restore_visual_deadzone();
    s_push_phase = MISSION_PUSH_IDLE;
    s_push_cursor = 0u;
    s_push_range_end = 0u;
    s_push_segment_end = 0u;
    s_push_expected_row = 0u;
    s_push_expected_col = 0u;
    s_pose_last_sequence = 0u;
    s_pose_sample_count = 0u;
    s_push_map_baseline_sequence = 0u;
    s_push_map_stable_count = 0u;
    s_push_move = '\0';
    (void)memset(&s_alignment, 0, sizeof(s_alignment));
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
    s_clear_frame_count = 0u;
    s_blast_event_index = 0u;
    s_segment_running = 0u;
    s_turn_complete = 0u;
    s_inspection_event = 0u;
    s_inspection_move_index = 0u;
    s_entry_retry_used = 0u;
    s_entry_alignment_done = 0u;
    mission_push_reset();
    label = 0u;
    (void)memset(&s_labels, 0, sizeof(s_labels));
    (void)memset(s_tracked_boxes, 0, sizeof(s_tracked_boxes));
    (void)memset(s_tracked_goals, 0, sizeof(s_tracked_goals));
    s_tracked_box_count = 0u;
    s_tracked_goal_count = 0u;
    (void)memset(&s_inspection_plan, 0, sizeof(s_inspection_plan));
    mission_solution_reset(&s_task_solution);
    mission_solution_reset(&s_segment_solution);
}

static void mission_enter_fatal(mission_fatal_fault_t fault,
                                sokoban_status_t status)
{
    s_last_status = status;
    s_fatal_fault = fault;
    s_timer_running = 0u;
    mission_push_reset();
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

    if((row >= SOKOBAN_MAP_HEIGHT) || (col >= SOKOBAN_MAP_WIDTH) ||
       (map[row][col] == '#') || (map[row][col] == '$') ||
       (map[row][col] == '*')) return 0u;
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

static uint8_t mission_initialize_labels(void)
{
    uint8_t row;
    uint8_t col;
    (void)memset(&s_labels, 0, sizeof(s_labels));
    (void)memset(s_tracked_boxes, 0, sizeof(s_tracked_boxes));
    (void)memset(s_tracked_goals, 0, sizeof(s_tracked_goals));
    s_tracked_box_count = 0u;
    s_tracked_goal_count = 0u;
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
                s_tracked_boxes[s_labels.box_count].active = 1u;
                s_tracked_boxes[s_labels.box_count].row = row;
                s_tracked_boxes[s_labels.box_count].col = col;
                s_tracked_boxes[s_labels.box_count].label =
                    s_labels.box_labels[s_labels.box_count];
                s_tracked_boxes[s_labels.box_count].bomb =
                    (uint8_t)(ch == '*');
                s_labels.box_count++;
                s_tracked_box_count++;
            }
            else if((ch == '.') || (ch == 'G') || (ch == '+'))
            {
                if(s_labels.goal_count >= SOKOBAN_MAX_BOXES) return 0u;
                s_tracked_goals[s_labels.goal_count].active = 1u;
                s_tracked_goals[s_labels.goal_count].row = row;
                s_tracked_goals[s_labels.goal_count].col = col;
                s_labels.goal_count++;
                s_tracked_goal_count++;
            }
        }
    }
    return 1u;
}

static void mission_restore_goal_under_player(uint8_t row, uint8_t col)
{
    uint8_t index;
    if(s_current_map[row][col] != '@') return;
    for(index = 0u; index < s_tracked_goal_count; index++)
    {
        if((s_tracked_goals[index].active != 0u) &&
           (s_tracked_goals[index].row == row) &&
           (s_tracked_goals[index].col == col))
        {
            s_current_map[row][col] = '+';
            return;
        }
    }
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
    s_player_row = row;
    s_player_col = col;
    mission_restore_goal_under_player(row, col);
    if(sequence == s_last_frame_sequence) return;
    s_last_frame_sequence = sequence;
    s_frame_updated = 1u;
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

static uint8_t mission_advance_expected_position(uint16_t begin,
                                                 uint16_t end)
{
    uint16_t index;
    int16_t row = s_push_expected_row;
    int16_t col = s_push_expected_col;

    for(index = begin; index < end; index++)
    {
        switch(s_task_solution.move_seq[index])
        {
            case 'U': row--; break;
            case 'D': row++; break;
            case 'L': col--; break;
            case 'R': col++; break;
            default: return 0u;
        }
        if((row < 0) || (row >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
           (col < 0) || (col >= (int16_t)SOKOBAN_MAP_WIDTH))
        {
            return 0u;
        }
    }

    s_push_expected_row = (uint8_t)row;
    s_push_expected_col = (uint8_t)col;
    return 1u;
}

static uint16_t mission_find_next_push(uint16_t begin, uint16_t end)
{
    uint16_t index;
    for(index = begin; index < end; index++)
    {
        if(sokoban_solution_move_is_push(&s_task_solution, index))
        {
            return index;
        }
    }
    return end;
}

static uint16_t mission_find_straight_push_end(uint16_t begin, uint16_t end)
{
    uint16_t index;
    char direction;
    if((begin >= end) ||
       !sokoban_solution_move_is_push(&s_task_solution, begin))
    {
        return begin;
    }
    direction = s_task_solution.move_seq[begin];
    index = (uint16_t)(begin + 1u);
    while((index < end) &&
          sokoban_solution_move_is_push(&s_task_solution, index) &&
          (s_task_solution.move_seq[index] == direction))
    {
        index++;
    }
    return index;
}

static uint8_t mission_push_flags_valid(void)
{
    uint16_t index;
    uint16_t count = 0u;
    for(index = 0u; index < s_task_solution.move_count; index++)
    {
        if(sokoban_solution_move_is_push(&s_task_solution, index))
        {
            count++;
        }
    }
    return (uint8_t)(count == s_task_solution.push_count);
}

static void mission_reset_pose_samples(void)
{
    camera_pose_snapshot_t pose;
    s_pose_sample_count = 0u;
    s_pose_last_sequence = 0u;
    if(camera_sokoban_copy_latest_pose(&pose))
    {
        s_pose_last_sequence = pose.sequence;
    }
}

static float mission_median_pose(float values[VISION_POSE_SAMPLE_COUNT])
{
    uint8_t outer;
    for(outer = 1u; outer < VISION_POSE_SAMPLE_COUNT; outer++)
    {
        float value = values[outer];
        uint8_t inner = outer;
        while((inner > 0u) && (values[inner - 1u] > value))
        {
            values[inner] = values[inner - 1u];
            inner--;
        }
        values[inner] = value;
    }
    return values[VISION_POSE_SAMPLE_COUNT / 2u];
}

static uint8_t mission_collect_stable_pose(float *pose_x, float *pose_y)
{
    camera_pose_snapshot_t pose;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    uint8_t index;

    if((mission_encoder_is_still() == 0u) ||
       (fabsf(Gyro.x) > VISION_POSE_MAX_GYRO_RATE))
    {
        s_pose_sample_count = 0u;
        return 0u;
    }
    if(!camera_sokoban_copy_latest_pose(&pose) ||
       (pose.sequence == s_pose_last_sequence))
    {
        return 0u;
    }
    s_pose_last_sequence = pose.sequence;

    if((pose.valid == 0u) ||
       (pose.confidence < VISION_POSE_MIN_CONFIDENCE))
    {
        return 0u;
    }

    s_pose_samples_x[s_pose_sample_count] = (float)pose.x100 / 100.0f;
    s_pose_samples_y[s_pose_sample_count] = (float)pose.y100 / 100.0f;
    s_pose_sample_count++;
    if(s_pose_sample_count < VISION_POSE_SAMPLE_COUNT)
    {
        return 0u;
    }

    min_x = max_x = s_pose_samples_x[0];
    min_y = max_y = s_pose_samples_y[0];
    for(index = 1u; index < VISION_POSE_SAMPLE_COUNT; index++)
    {
        if(s_pose_samples_x[index] < min_x) min_x = s_pose_samples_x[index];
        if(s_pose_samples_x[index] > max_x) max_x = s_pose_samples_x[index];
        if(s_pose_samples_y[index] < min_y) min_y = s_pose_samples_y[index];
        if(s_pose_samples_y[index] > max_y) max_y = s_pose_samples_y[index];
    }
    if(((max_x - min_x) > VISION_POSE_MAX_SPREAD_CELL) ||
       ((max_y - min_y) > VISION_POSE_MAX_SPREAD_CELL))
    {
        s_pose_samples_x[0] =
            s_pose_samples_x[VISION_POSE_SAMPLE_COUNT - 1u];
        s_pose_samples_y[0] =
            s_pose_samples_y[VISION_POSE_SAMPLE_COUNT - 1u];
        s_pose_sample_count = 1u;
        return 0u;
    }

    *pose_x = mission_median_pose(s_pose_samples_x);
    *pose_y = mission_median_pose(s_pose_samples_y);
    return 1u;
}

static uint8_t mission_start_push_segment(uint16_t begin,
                                          uint16_t end,
                                          mission_push_phase_t wait_phase)
{
    if((begin >= end) || !mission_start_segment(&s_task_solution, begin, end))
    {
        return 0u;
    }
    s_push_segment_end = end;
    s_push_phase = wait_phase;
    return 1u;
}

static uint8_t mission_begin_push_range(uint16_t begin,
                                        uint16_t end,
                                        uint8_t start_row,
                                        uint8_t start_col)
{
    if((begin > end) || (end > s_task_solution.move_count) ||
       (start_row >= SOKOBAN_MAP_HEIGHT) ||
       (start_col >= SOKOBAN_MAP_WIDTH) ||
       !mission_push_flags_valid())
    {
        return 0u;
    }
    s_push_cursor = begin;
    s_push_range_end = end;
    s_push_segment_end = begin;
    s_push_expected_row = start_row;
    s_push_expected_col = start_col;
    s_push_phase = MISSION_PUSH_PREPARE;
    return 1u;
}

static uint16_t mission_find_straight_walk_end(uint16_t begin, uint16_t end)
{
    uint16_t index = begin;
    char direction;
    if(begin >= end)
    {
        return begin;
    }
    direction = s_task_solution.move_seq[begin];
    while((index < end) &&
          (s_task_solution.move_seq[index] == direction))
    {
        index++;
    }
    return index;
}

static uint8_t mission_start_visual_correction(float error,
                                               uint8_t x_axis)
{
    float distance = fabsf(error) * VISION_CORRECTION_CELL_DISTANCE;
    float max_distance = (s_alignment.long_straight != 0u) ?
                         VISION_LONG_CORRECTION_MAX_DISTANCE :
                         VISION_CORRECTION_MAX_DISTANCE;
    sokoban_direction_t world_direction;
    sokoban_body_direction_t body_direction;

    if(distance < VISION_CORRECTION_MIN_DISTANCE)
    {
        distance = VISION_CORRECTION_MIN_DISTANCE;
    }
    if(distance > max_distance)
    {
        distance = max_distance;
    }
    if(x_axis != 0u)
    {
        world_direction = (error > 0.0f) ? SOKOBAN_DIR_RIGHT :
                                           SOKOBAN_DIR_LEFT;
    }
    else
    {
        world_direction = (error > 0.0f) ? SOKOBAN_DIR_DOWN :
                                           SOKOBAN_DIR_UP;
    }
    body_direction = sokoban_world_to_body(s_heading, world_direction);
    if(!path_executor_clear_carry_error())
    {
        return 0u;
    }
    s_visual_deadzone_saved = MecanumGetSpeedDeadzoneMinRatio();
    MecanumSetSpeedDeadzoneMinRatio(VISION_CORRECTION_DEADZONE_RATIO);
    s_visual_deadzone_active = 1u;
    if(!path_executor_start_body_step_with_distance_and_tolerance(
           body_direction,
           distance,
           VISION_CORRECTION_PATH_TOLERANCE))
    {
        mission_restore_visual_deadzone();
        return 0u;
    }
    s_alignment.last_correction_x_axis = x_axis;
    s_alignment.last_correction_sign = (error > 0.0f) ? 1 : -1;
    return 1u;
}

static uint8_t mission_alignment_begin(uint8_t target_row,
                                       uint8_t target_col,
                                       uint8_t prefer_x,
                                       uint8_t skip_on_timeout)
{
    if((target_row == 0u) || (target_row + 1u >= SOKOBAN_MAP_HEIGHT) ||
       (target_col == 0u) || (target_col + 1u >= SOKOBAN_MAP_WIDTH))
    {
        return 0u;
    }
    s_alignment.active = 1u;
    s_alignment.moving = 0u;
    s_alignment.target_row = target_row;
    s_alignment.target_col = target_col;
    s_alignment.prefer_x = prefer_x;
    s_alignment.correction_count = 0u;
    s_alignment.skip_on_timeout = skip_on_timeout;
    s_alignment.long_straight = 0u;
    s_alignment.last_correction_x_axis = 0u;
    s_alignment.last_correction_sign = 0;
    s_alignment.wait_10ms = 0u;
    s_alignment.elapsed_10ms = 0u;
    mission_reset_pose_samples();
    MecanumCarStop();
    return 1u;
}

static uint8_t mission_alignment_begin_after_walk(uint8_t target_row,
                                                  uint8_t target_col,
                                                  uint16_t walk_cells)
{
    if(!mission_alignment_begin(target_row, target_col, 1u, 1u))
    {
        return 0u;
    }
    s_alignment.long_straight =
        (uint8_t)(walk_cells >= MISSION_LONG_STRAIGHT_MIN_CELLS);
    return 1u;
}

static mission_align_result_t mission_alignment_update(void)
{
    float pose_x;
    float pose_y;
    float error_x;
    float error_y;
    float replan_error = (s_alignment.long_straight != 0u) ?
                         VISION_LONG_CORRECTION_MAX_ERROR_CELL :
                         VISION_REPLAN_ERROR_CELL;

    if(s_alignment.active == 0u)
    {
        return MISSION_ALIGN_FAULT;
    }
    if((s_alignment.skip_on_timeout != 0u) &&
       (s_alignment.elapsed_10ms >= VISION_ALIGNMENT_TOTAL_TIMEOUT_10MS))
    {
        mission_restore_visual_deadzone();
        path_executor_abort();
        s_alignment.moving = 0u;
        s_alignment.active = 0u;
        mission_reset_pose_samples();
        MecanumCarStop();
        return MISSION_ALIGN_DONE;
    }
    if(s_alignment.moving != 0u)
    {
        if(path_executor_is_fault())
        {
            path_executor_fault_reason_t reason =
                path_executor_get_fault_reason();
            mission_restore_visual_deadzone();
            s_alignment.moving = 0u;
            if((s_alignment.skip_on_timeout != 0u) &&
               ((reason == PATH_EXECUTOR_FAULT_RUN_TIMEOUT) ||
                (reason == PATH_EXECUTOR_FAULT_STOP_TIMEOUT) ||
                (reason == PATH_EXECUTOR_FAULT_YAW_TIMEOUT)))
            {
                path_executor_abort();
                s_alignment.active = 0u;
                mission_reset_pose_samples();
                MecanumCarStop();
                return MISSION_ALIGN_DONE;
            }
            return MISSION_ALIGN_FAULT;
        }
        if(path_executor_is_done())
        {
            mission_restore_visual_deadzone();
            s_alignment.moving = 0u;
            s_alignment.wait_10ms = 0u;
            mission_reset_pose_samples();
            MecanumCarStop();
        }
        return MISSION_ALIGN_WAITING;
    }
    if(!mission_collect_stable_pose(&pose_x, &pose_y))
    {
        if(s_alignment.wait_10ms < VISION_POSE_TIMEOUT_10MS)
        {
            s_alignment.wait_10ms++;
        }
        if((s_alignment.skip_on_timeout != 0u) &&
           (s_alignment.wait_10ms >= VISION_POSE_TIMEOUT_10MS))
        {
            s_alignment.active = 0u;
            mission_reset_pose_samples();
            MecanumCarStop();
            return MISSION_ALIGN_DONE;
        }
        return MISSION_ALIGN_WAITING;
    }
    s_alignment.wait_10ms = 0u;

    error_x = (float)s_alignment.target_col - pose_x;
    error_y = (float)s_alignment.target_row - pose_y;
    if((fabsf(error_x) > replan_error) ||
       (fabsf(error_y) > replan_error))
    {
        int16_t nearest_col = (int16_t)(pose_x + 0.5f);
        int16_t nearest_row = (int16_t)(pose_y + 0.5f);
        char nearest_cell;
        if((nearest_row <= 0) ||
           (nearest_row + 1 >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
           (nearest_col <= 0) ||
           (nearest_col + 1 >= (int16_t)SOKOBAN_MAP_WIDTH))
        {
            return MISSION_ALIGN_FAULT;
        }
        nearest_cell = s_current_map[nearest_row][nearest_col];
        if((nearest_cell == '#') || (nearest_cell == '$') ||
           (nearest_cell == '*'))
        {
            return MISSION_ALIGN_FAULT;
        }
        if(((uint8_t)nearest_row != s_alignment.target_row) ||
           ((uint8_t)nearest_col != s_alignment.target_col))
        {
            s_player_row = (uint8_t)nearest_row;
            s_player_col = (uint8_t)nearest_col;
            s_alignment.active = 0u;
            MecanumCarStop();
            return MISSION_ALIGN_REPLAN;
        }
    }
    if((s_alignment.correction_count != 0u) &&
       (s_alignment.last_correction_sign != 0))
    {
        float last_axis_error = (s_alignment.last_correction_x_axis != 0u) ?
                                error_x : error_y;
        if((last_axis_error * (float)s_alignment.last_correction_sign < 0.0f) &&
           (fabsf(last_axis_error) <=
            VISION_CORRECTION_REVERSE_GUARD_CELL))
        {
            if(s_alignment.last_correction_x_axis != 0u)
            {
                error_x = 0.0f;
            }
            else
            {
                error_y = 0.0f;
            }
        }
    }
    if(((fabsf(error_x) <= VISION_CORRECTION_DEADBAND_CELL) &&
        (fabsf(error_y) <= VISION_CORRECTION_DEADBAND_CELL)) ||
       ((s_alignment.correction_count != 0u) &&
        (fabsf(error_x) <= VISION_CORRECTION_FINAL_CELL) &&
        (fabsf(error_y) <= VISION_CORRECTION_FINAL_CELL)))
    {
        s_alignment.active = 0u;
        MecanumCarStop();
        return MISSION_ALIGN_DONE;
    }
    if(s_alignment.correction_count >= VISION_CORRECTION_MAX_ATTEMPTS)
    {
        s_alignment.active = 0u;
        MecanumCarStop();
        return MISSION_ALIGN_DONE;
    }
    if(s_alignment.prefer_x != 0u)
    {
        if(fabsf(error_x) > VISION_CORRECTION_DEADBAND_CELL)
        {
            if(!mission_start_visual_correction(error_x, 1u))
            {
                return MISSION_ALIGN_FAULT;
            }
        }
        else if(!mission_start_visual_correction(error_y, 0u))
        {
            return MISSION_ALIGN_FAULT;
        }
    }
    else if(fabsf(error_y) > VISION_CORRECTION_DEADBAND_CELL)
    {
        if(!mission_start_visual_correction(error_y, 0u))
        {
            return MISSION_ALIGN_FAULT;
        }
    }
    else if(!mission_start_visual_correction(error_x, 1u))
    {
        return MISSION_ALIGN_FAULT;
    }
    s_alignment.moving = 1u;
    s_alignment.correction_count++;
    return MISSION_ALIGN_WAITING;
}

static char mission_map_object(char ch)
{
    if(ch == '@') return '-';
    if(ch == '+') return '.';
    return ch;
}

static uint8_t mission_object_maps_equal(
    const char left[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    const char right[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8_t row;
    uint8_t col;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            if(mission_map_object(left[row][col]) !=
               mission_map_object(right[row][col]))
            {
                return 0u;
            }
        }
    }
    return 1u;
}

static uint8_t mission_cell_has_box(char ch)
{
    return (uint8_t)((ch == '$') || (ch == '*'));
}

static uint8_t mission_push_map_is_expected(void)
{
    char before_source =
        mission_map_object(s_push_before_map[s_push_source_row][s_push_source_col]);
    char before_target =
        mission_map_object(s_push_before_map[s_push_target_row][s_push_target_col]);
    char after_source =
        mission_map_object(s_current_map[s_push_source_row][s_push_source_col]);
    char after_target =
        mission_map_object(s_current_map[s_push_target_row][s_push_target_col]);

    if(mission_cell_has_box(after_source))
    {
        return 0u;
    }
    if((before_source == '*') && (before_target == '#'))
    {
        return (uint8_t)(mission_object_maps_equal(s_push_before_map,
                                                   s_current_map) == 0u);
    }
    if((before_target == '.') || (before_target == 'G'))
    {
        return (uint8_t)(!mission_cell_has_box(after_target) &&
                         (after_target != '.') && (after_target != 'G'));
    }
    return mission_cell_has_box(after_target);
}

static void mission_update_tracked_push(void)
{
    uint8_t index;
    uint8_t goal_index = SOKOBAN_INVALID_CELL;
    for(index = 0u; index < s_tracked_goal_count; index++)
    {
        if((s_tracked_goals[index].active != 0u) &&
           (s_tracked_goals[index].row == s_push_target_row) &&
           (s_tracked_goals[index].col == s_push_target_col))
        {
            goal_index = index;
            break;
        }
    }
    for(index = 0u; index < s_tracked_box_count; index++)
    {
        if((s_tracked_boxes[index].active != 0u) &&
           (s_tracked_boxes[index].row == s_push_source_row) &&
           (s_tracked_boxes[index].col == s_push_source_col))
        {
            if((goal_index != SOKOBAN_INVALID_CELL) ||
               ((s_tracked_boxes[index].bomb != 0u) &&
                (mission_map_object(
                    s_push_before_map[s_push_target_row][s_push_target_col]) ==
                 '#')))
            {
                if((s_tracked_boxes[index].bomb != 0u) &&
                   (mission_map_object(
                       s_push_before_map[s_push_target_row][s_push_target_col]) ==
                    '#'))
                {
                    s_blast_event_index++;
                }
                s_tracked_boxes[index].active = 0u;
                if(goal_index != SOKOBAN_INVALID_CELL)
                {
                    s_tracked_goals[goal_index].active = 0u;
                }
            }
            else
            {
                s_tracked_boxes[index].row = s_push_target_row;
                s_tracked_boxes[index].col = s_push_target_col;
            }
            break;
        }
    }
}

static uint8_t mission_rebuild_labels(void)
{
    sokoban_label_table_t rebuilt;
    uint8_t row;
    uint8_t col;
    uint8_t index;
    (void)memset(&rebuilt, 0, sizeof(rebuilt));
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = mission_map_object(s_current_map[row][col]);
            if((ch == '$') || (ch == '*'))
            {
                if(rebuilt.box_count >= SOKOBAN_MAX_BOXES) return 0u;
                for(index = 0u; index < s_tracked_box_count; index++)
                {
                    if((s_tracked_boxes[index].active != 0u) &&
                       (s_tracked_boxes[index].row == row) &&
                       (s_tracked_boxes[index].col == col))
                    {
                        rebuilt.box_labels[rebuilt.box_count] =
                            s_tracked_boxes[index].label;
                        break;
                    }
                }
                if(index >= s_tracked_box_count) return 0u;
                rebuilt.box_count++;
            }
            else if((ch == '.') || (ch == 'G'))
            {
                if(rebuilt.goal_count >= SOKOBAN_MAX_BOXES) return 0u;
                for(index = 0u; index < s_tracked_goal_count; index++)
                {
                    if((s_tracked_goals[index].active != 0u) &&
                       (s_tracked_goals[index].row == row) &&
                       (s_tracked_goals[index].col == col))
                    {
                        rebuilt.goal_labels[rebuilt.goal_count] =
                            s_tracked_goals[index].label;
                        break;
                    }
                }
                if(index >= s_tracked_goal_count) return 0u;
                rebuilt.goal_count++;
            }
        }
    }
    s_labels = rebuilt;
    return 1u;
}

static uint8_t mission_replan_current(void)
{
    mission_solution_reset(&s_task_solution);
    if(s_current_level == 1u)
    {
        s_last_status = sokoban_solve_decomposed(s_current_map,
                                                 &s_task_solution);
        if(s_last_status != SOKOBAN_STATUS_OK)
        {
            s_last_status = sokoban_solve_bidirectional_astar(
                s_current_map, &s_task_solution);
        }
    }
    else
    {
        if(!mission_rebuild_labels())
        {
            s_last_status = SOKOBAN_STATUS_INVALID_MAP;
            return 0u;
        }
        s_last_status = sokoban_solve_labeled(s_current_map,
                                              s_player_row,
                                              s_player_col,
                                              &s_labels,
                                              &s_task_solution);
    }
    if((s_last_status != SOKOBAN_STATUS_OK) ||
       !mission_push_flags_valid())
    {
        return 0u;
    }
    return mission_begin_push_range(0u, s_task_solution.move_count,
                                    s_player_row, s_player_col);
}

static uint8_t mission_restart_after_reanchor(void)
{
    if(s_state == MISSION_EXECUTE_INSPECTION)
    {
        (void)memcpy(s_initial_map, s_current_map, sizeof(s_initial_map));
        s_entry_row = s_player_row;
        s_entry_col = s_player_col;
        mission_push_reset();
        s_segment_running = 0u;
        mission_set_state(MISSION_PLAN_INSPECTION);
        return 1u;
    }
    if((s_state == MISSION_EXECUTE_NORMAL_RETURN_ENTRY) ||
       (s_state == MISSION_NORMAL_RETURN_PHYSICAL_START))
    {
        mission_push_reset();
        mission_set_state(MISSION_PLAN_NORMAL_RETURN_ENTRY);
        return 1u;
    }
    if((s_state == MISSION_EXECUTE_ABORT_RETURN_ENTRY) ||
       (s_state == MISSION_ABORT_RETURN_PHYSICAL_START))
    {
        mission_push_reset();
        mission_set_state(MISSION_PLAN_ABORT_RETURN_ENTRY);
        return 1u;
    }
    if(!mission_replan_current() ||
       !mission_alignment_begin(s_player_row, s_player_col, 1u, 1u))
    {
        return 0u;
    }
    s_push_phase = MISSION_PUSH_WAIT_WALK_POSE;
    return 1u;
}

static uint8_t mission_prepare_push_transaction(uint16_t push_end)
{
    int16_t source_row = s_push_expected_row;
    int16_t source_col = s_push_expected_col;
    int16_t target_row;
    int16_t target_col;
    uint16_t index;
    if((push_end <= s_push_cursor) || (push_end > s_push_range_end))
    {
        return 0u;
    }
    s_push_move = s_task_solution.move_seq[s_push_cursor];
    switch(s_push_move)
    {
        case 'U': source_row--; break;
        case 'D': source_row++; break;
        case 'L': source_col--; break;
        case 'R': source_col++; break;
        default: return 0u;
    }
    target_row = source_row;
    target_col = source_col;
    for(index = s_push_cursor; index < push_end; index++)
    {
        if(!sokoban_solution_move_is_push(&s_task_solution, index) ||
           (s_task_solution.move_seq[index] != s_push_move))
        {
            return 0u;
        }
        switch(s_push_move)
        {
            case 'U': target_row--; break;
            case 'D': target_row++; break;
            case 'L': target_col--; break;
            case 'R': target_col++; break;
            default: return 0u;
        }
    }
    if((source_row < 0) || (source_row >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
       (source_col < 0) || (source_col >= (int16_t)SOKOBAN_MAP_WIDTH) ||
       (target_row < 0) || (target_row >= (int16_t)SOKOBAN_MAP_HEIGHT) ||
       (target_col < 0) || (target_col >= (int16_t)SOKOBAN_MAP_WIDTH))
    {
        return 0u;
    }
    s_push_source_row = (uint8_t)source_row;
    s_push_source_col = (uint8_t)source_col;
    s_push_target_row = (uint8_t)target_row;
    s_push_target_col = (uint8_t)target_col;
    if(!mission_cell_has_box(
           mission_map_object(s_current_map[s_push_source_row][s_push_source_col])))
    {
        return 0u;
    }
    (void)memcpy(s_push_before_map, s_current_map, sizeof(s_push_before_map));
    return 1u;
}

static mission_push_update_result_t mission_update_push_range(void)
{
    mission_align_result_t align_result;
    switch(s_push_phase)
    {
        case MISSION_PUSH_PREPARE:
        {
            uint16_t push_index;
            uint16_t walk_end;
            if(s_push_cursor >= s_push_range_end)
            {
                s_push_phase = MISSION_PUSH_IDLE;
                return MISSION_PUSH_UPDATE_DONE;
            }
            push_index = mission_find_next_push(s_push_cursor,
                                                s_push_range_end);
            if(push_index > s_push_cursor)
            {
                walk_end = mission_find_straight_walk_end(s_push_cursor,
                                                          push_index);
                if(!mission_start_push_segment(s_push_cursor, walk_end,
                                               MISSION_PUSH_WAIT_WALK))
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
            }
            else if(push_index == s_push_cursor)
            {
                char move = s_task_solution.move_seq[s_push_cursor];
                uint8_t prefer_x = (uint8_t)((move == 'U') || (move == 'D'));
                if(!mission_alignment_begin(s_push_expected_row,
                                            s_push_expected_col,
                                            prefer_x, 1u))
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                s_push_phase = MISSION_PUSH_WAIT_POSE;
            }
            else
            {
                return MISSION_PUSH_UPDATE_FAULT;
            }
        } break;

        case MISSION_PUSH_WAIT_WALK:
            if(path_executor_is_done())
            {
                uint16_t walk_cells =
                    (uint16_t)(s_push_segment_end - s_push_cursor);
                if(!mission_advance_expected_position(s_push_cursor,
                                                      s_push_segment_end))
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                s_push_cursor = s_push_segment_end;
                if(!mission_alignment_begin_after_walk(
                       s_push_expected_row, s_push_expected_col, walk_cells))
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                s_push_phase = MISSION_PUSH_WAIT_WALK_POSE;
            }
            break;

        case MISSION_PUSH_WAIT_WALK_POSE:
            align_result = mission_alignment_update();
            if(align_result == MISSION_ALIGN_FAULT)
            {
                return MISSION_PUSH_UPDATE_FAULT;
            }
            if(align_result == MISSION_ALIGN_REPLAN)
            {
                if(!mission_restart_after_reanchor())
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                break;
            }
            if(align_result == MISSION_ALIGN_DONE)
            {
                s_push_phase = MISSION_PUSH_PREPARE;
            }
            break;

        case MISSION_PUSH_WAIT_POSE:
            align_result = mission_alignment_update();
            if(align_result == MISSION_ALIGN_FAULT)
            {
                return MISSION_PUSH_UPDATE_FAULT;
            }
            if(align_result == MISSION_ALIGN_REPLAN)
            {
                if(!mission_restart_after_reanchor())
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                break;
            }
            if(align_result == MISSION_ALIGN_DONE)
            {
                uint16_t push_end =
                    mission_find_straight_push_end(s_push_cursor,
                                                   s_push_range_end);
                if(!mission_prepare_push_transaction(push_end) ||
                   !mission_start_push_segment(s_push_cursor,
                                               push_end,
                                               MISSION_PUSH_WAIT_PUSH))
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
            }
            break;

        case MISSION_PUSH_WAIT_PUSH:
            if(path_executor_is_done())
            {
                MecanumCarStop();
                s_push_map_baseline_sequence = s_last_frame_sequence;
                s_push_map_stable_count = 0u;
                s_push_phase = MISSION_PUSH_WAIT_MAP;
            }
            break;

        case MISSION_PUSH_WAIT_MAP:
            if((s_frame_updated != 0u) &&
               (s_last_frame_sequence > s_push_map_baseline_sequence))
            {
                if((s_push_map_stable_count != 0u) &&
                   mission_object_maps_equal(s_push_candidate_map,
                                             s_current_map))
                {
                    if(s_push_map_stable_count < MISSION_MAP_STABLE_FRAMES)
                    {
                        s_push_map_stable_count++;
                    }
                }
                else
                {
                    (void)memcpy(s_push_candidate_map, s_current_map,
                                 sizeof(s_push_candidate_map));
                    s_push_map_stable_count = 1u;
                }
            }
            if(s_push_map_stable_count >= MISSION_MAP_STABLE_FRAMES)
            {
                if(mission_push_map_is_expected())
                {
                    mission_update_tracked_push();
                    if(mission_task_complete(s_current_map))
                    {
                        s_push_phase = MISSION_PUSH_IDLE;
                        return MISSION_PUSH_UPDATE_DONE;
                    }
                    if(!mission_replan_current())
                    {
                        return MISSION_PUSH_UPDATE_FAULT;
                    }
                    if(!mission_alignment_begin(s_player_row, s_player_col,
                                                1u, 1u))
                    {
                        return MISSION_PUSH_UPDATE_FAULT;
                    }
                    s_push_phase = MISSION_PUSH_WAIT_WALK_POSE;
                }
                else if(mission_object_maps_equal(s_push_before_map,
                                                  s_current_map))
                {
                    if(!mission_alignment_begin(s_push_expected_row,
                                                s_push_expected_col,
                                                1u, 1u))
                    {
                        return MISSION_PUSH_UPDATE_FAULT;
                    }
                    s_push_phase = MISSION_PUSH_WAIT_RETRY_POSE;
                }
                else
                {
                    if(!mission_map_structurally_valid(s_current_map,
                                                       s_player_row,
                                                       s_player_col))
                    {
                        return MISSION_PUSH_UPDATE_FAULT;
                    }
                    if(s_current_level == 1u)
                    {
                        if(!mission_replan_current() ||
                           !mission_alignment_begin(s_player_row,
                                                     s_player_col, 1u, 1u))
                        {
                            return MISSION_PUSH_UPDATE_FAULT;
                        }
                        s_push_phase = MISSION_PUSH_WAIT_WALK_POSE;
                    }
                    else
                    {
                        (void)memcpy(s_initial_map, s_current_map,
                                     sizeof(s_initial_map));
                        s_entry_row = s_player_row;
                        s_entry_col = s_player_col;
                        mission_push_reset();
                        s_segment_running = 0u;
                        mission_set_state(MISSION_PLAN_INSPECTION);
                    }
                }
            }
            break;

        case MISSION_PUSH_WAIT_RETRY_POSE:
            align_result = mission_alignment_update();
            if(align_result == MISSION_ALIGN_FAULT)
            {
                return MISSION_PUSH_UPDATE_FAULT;
            }
            if(align_result == MISSION_ALIGN_REPLAN)
            {
                if(!mission_restart_after_reanchor())
                {
                    return MISSION_PUSH_UPDATE_FAULT;
                }
                break;
            }
            if(align_result == MISSION_ALIGN_DONE)
            {
                s_push_phase = MISSION_PUSH_PREPARE;
            }
            break;

        default:
            return MISSION_PUSH_UPDATE_FAULT;
    }
    return MISSION_PUSH_UPDATE_RUNNING;
}

static void mission_start_return(void)
{
    s_segment_running = 0u;
    mission_push_reset();
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
    mission_push_reset();
    path_executor_abort();
    MecanumCarStop();
    mission_start_return();
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
    if(s_entry_alignment_done == 0u)
    {
        if(!mission_alignment_begin(s_player_row, s_player_col, 1u, 1u))
        {
            mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            return;
        }
        mission_set_state(MISSION_ALIGN_ENTRY);
        return;
    }
    s_have_valid_map = 1u;
    s_entry_row = s_player_row;
    s_entry_col = s_player_col;
    (void)memcpy(s_initial_map, s_current_map, sizeof(s_initial_map));
    if(s_current_level == 1u)
    {
        if(!mission_initialize_labels())
        {
            mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            return;
        }
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
                     mission_object_maps_equal(s_candidate_map,
                                               s_current_map));
    if(same != 0u)
    {
        s_candidate_player_row = s_player_row;
        s_candidate_player_col = s_player_col;
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
    s_last_status = SOKOBAN_STATUS_OK;
    mission_reset_level_data();
    path_executor_set_heading(s_heading);
    MecanumCarStop();
}

void mission_controller_update_10ms(void)
{
    s_state_elapsed_10ms++;
    if(s_timer_running != 0u) s_elapsed_10ms++;
    if((s_alignment.active != 0u) &&
       (s_alignment.skip_on_timeout != 0u) &&
       (s_alignment.elapsed_10ms < VISION_ALIGNMENT_TOTAL_TIMEOUT_10MS))
    {
        s_alignment.elapsed_10ms++;
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
            MecanumCarStop();
            if(mission_start_one_move(
                   'R', (s_current_level == 1u) ?
                        MISSION_FIRST_ENTRY_DISTANCE :
                        MISSION_ENTRY_CELL_DISTANCE))
            {
                s_timer_running = 1u;
                mission_reset_map_acquisition();
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
                mission_reset_map_acquisition();
                mission_set_state(MISSION_WAIT_VALID_MAP);
            }
            break;

        case MISSION_ENTRY_RETRY_STEP:
            if(path_executor_is_done())
            {
                MecanumCarStop();
                mission_reset_map_acquisition();
                mission_set_state(MISSION_WAIT_VALID_MAP);
            }
            break;

        case MISSION_WAIT_VALID_MAP:
            mission_update_map_stability();
            if((s_state == MISSION_WAIT_VALID_MAP) &&
               (s_current_level > 1u) &&
               (s_entry_retry_used == 0u) &&
               (s_map_stable_frames == 0u) &&
               (s_state_elapsed_10ms >= MISSION_ENTRY_MAP_GRACE_10MS))
            {
                if(mission_start_one_move('R',
                                          MISSION_ENTRY_CELL_DISTANCE))
                {
                    s_entry_retry_used = 1u;
                    mission_set_state(MISSION_ENTRY_RETRY_STEP);
                }
                else
                {
                    mission_enter_fatal(MISSION_FATAL_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            break;

        case MISSION_ALIGN_ENTRY:
        {
            mission_align_result_t align_result = mission_alignment_update();
            if(align_result == MISSION_ALIGN_FAULT)
            {
                mission_enter_fatal(MISSION_FATAL_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
            else if(align_result == MISSION_ALIGN_REPLAN)
            {
                if(!mission_alignment_begin(s_player_row, s_player_col,
                                            1u, 1u))
                {
                    mission_enter_fatal(MISSION_FATAL_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(align_result == MISSION_ALIGN_DONE)
            {
                s_entry_alignment_done = 1u;
                mission_reset_map_acquisition();
                mission_set_state(MISSION_WAIT_VALID_MAP);
            }
        } break;

        case MISSION_PLAN_PLAIN_PUSH:
            mission_solution_reset(&s_task_solution);
            s_last_status = sokoban_solve_decomposed(s_initial_map,
                                                      &s_task_solution);
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                s_last_status = sokoban_solve_bidirectional_astar(
                    s_initial_map, &s_task_solution);
            }
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                mission_abort_level(s_last_status);
            }
            else if(mission_begin_push_range(0u,
                                             s_task_solution.move_count,
                                             s_entry_row,
                                             s_entry_col))
            {
                s_segment_running = 1u;
                mission_set_state(MISSION_EXECUTE_PUSH);
            }
            else
            {
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
                else
                {
                    s_task_solution = s_inspection_plan.route;
                    if(mission_begin_push_range(s_inspection_move_index, end,
                                                s_player_row, s_player_col))
                    {
                        s_segment_running = 1u;
                    }
                    else
                    {
                        mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
                    }
                }
            }
            else
            {
                mission_push_update_result_t push_result =
                    mission_update_push_range();
                if(push_result == MISSION_PUSH_UPDATE_FAULT)
                {
                    mission_enter_fatal(MISSION_FATAL_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
                else if(push_result == MISSION_PUSH_UPDATE_DONE)
                {
                    s_segment_running = 0u;
                    s_inspection_move_index =
                        s_inspection_plan.events[s_inspection_event].move_index;
                    mission_set_state(MISSION_WAIT_TURN);
                }
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
            s_last_status = sokoban_solve_labeled(
                s_initial_map,
                s_inspection_plan.final_row,
                s_inspection_plan.final_col,
                &s_labels,
                &s_task_solution);
            if(s_last_status != SOKOBAN_STATUS_OK)
            {
                mission_abort_level(s_last_status);
            }
            else
            {
                if(s_task_solution.move_count == 0u)
                {
                    mission_set_state(MISSION_WAIT_MAP_CLEAR);
                }
                else if(mission_begin_push_range(
                            0u, s_task_solution.move_count,
                            s_inspection_plan.final_row,
                            s_inspection_plan.final_col))
                {
                    s_segment_running = 1u;
                    mission_set_state(MISSION_EXECUTE_PUSH);
                }
                else
                {
                    mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            break;

        case MISSION_EXECUTE_PUSH:
            {
                mission_push_update_result_t push_result =
                    mission_update_push_range();
                if(push_result == MISSION_PUSH_UPDATE_FAULT)
                {
                    mission_enter_fatal(MISSION_FATAL_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
                else if(push_result == MISSION_PUSH_UPDATE_DONE)
                {
                    s_segment_running = 0u;
                    s_clear_frame_count = 0u;
                    mission_set_state(MISSION_WAIT_MAP_CLEAR);
                }
            }
            break;

        case MISSION_WAIT_BLAST_MAP:
            mission_enter_fatal(MISSION_FATAL_PATH,
                                SOKOBAN_STATUS_INVALID_MAP);
            break;

        case MISSION_WAIT_MAP_CLEAR:
            if(s_frame_updated != 0u)
            {
                if(mission_task_complete(s_current_map))
                {
                    if(s_clear_frame_count < MISSION_MAP_STABLE_FRAMES)
                    {
                        s_clear_frame_count++;
                    }
                }
                else
                {
                    s_clear_frame_count = 0u;
                }
            }
            if(s_clear_frame_count >= MISSION_MAP_STABLE_FRAMES)
            {
                s_level_result = MISSION_LEVEL_RESULT_SUCCESS;
                mission_start_return();
            }
            break;

        case MISSION_PLAN_NORMAL_RETURN_ENTRY:
        case MISSION_PLAN_ABORT_RETURN_ENTRY:
        {
            sokoban_status_t return_status = sokoban_plan_walk(
                s_current_map, s_player_row, s_player_col,
                MISSION_RETURN_ROW, MISSION_RETURN_COL, &s_task_solution);
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
            else if(s_level_result != MISSION_LEVEL_RESULT_ABORTED)
            {
                if(mission_start_segment(&s_task_solution, 0u,
                                         s_task_solution.move_count))
                {
                    s_segment_running = 1u;
                    mission_set_state(MISSION_EXECUTE_NORMAL_RETURN_ENTRY);
                }
                else
                {
                    mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(mission_begin_push_range(0u,
                                             s_task_solution.move_count,
                                             s_player_row,
                                             s_player_col))
            {
                mission_set_state(MISSION_EXECUTE_ABORT_RETURN_ENTRY);
            }
            else
            {
                mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                    SOKOBAN_STATUS_INVALID_MAP);
            }
        } break;

        case MISSION_EXECUTE_NORMAL_RETURN_ENTRY:
            if(path_executor_is_done())
            {
                s_segment_running = 0u;
                mission_set_state(MISSION_NORMAL_RETURN_PHYSICAL_START);
            }
            break;

        case MISSION_EXECUTE_ABORT_RETURN_ENTRY:
            {
                mission_push_update_result_t push_result =
                    mission_update_push_range();
                if(push_result == MISSION_PUSH_UPDATE_FAULT)
                {
                    mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
                else if(push_result == MISSION_PUSH_UPDATE_DONE)
                {
                    s_segment_running = 0u;
                    mission_set_state(MISSION_ABORT_RETURN_PHYSICAL_START);
                }
            }
            break;

        case MISSION_NORMAL_RETURN_PHYSICAL_START:
        case MISSION_ABORT_RETURN_PHYSICAL_START:
            if(s_segment_running == 0u)
            {
                if(mission_start_one_move('L', MISSION_RETURN_DISTANCE))
                {
                    s_segment_running = 1u;
                }
                else
                {
                    mission_enter_fatal(MISSION_FATAL_RETURN_PATH,
                                        SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(path_executor_is_done() ||
                    (s_state_elapsed_10ms >= MISSION_RETURN_ZONE_TIMEOUT_10MS))
            {
                path_executor_abort();
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
        if(object_index < s_tracked_box_count)
        {
            s_tracked_boxes[object_index].label = value;
        }
        return 1u;
    }
    if((type == SOKOBAN_OBJECT_GOAL) &&
       (object_index < s_labels.goal_count))
    {
        s_labels.goal_labels[object_index] = value;
        if(object_index < s_tracked_goal_count)
        {
            s_tracked_goals[object_index].label = value;
        }
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

uint8_t mission_controller_should_stop(void)
{
    return (uint8_t)((s_state == MISSION_BOOT_DELAY) ||
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
    static uint8_t digit_backoff_count = 0u;
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
        digit_backoff_count = 0u;
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
            if(digit_backoff_count >= DIGIT_RETRY_BACKOFF_MAX_COUNT)
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

            digit_backoff_count++;
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
    return s_task_solution.blast_count;
}
