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
#define MISSION_BLAST_STABLE_FRAMES          (2u)
#define MISSION_STILL_ENCODER_LIMIT          (2)

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
static uint8_t s_blast_match_count = 0u;
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
static float s_entry_distance = 24.0f;

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
    s_blast_match_count = 0u;
    s_blast_event_index = 0u;
    s_segment_running = 0u;
    s_turn_complete = 0u;
    s_inspection_event = 0u;
    s_inspection_move_index = 0u;
    label = 0u;
    (void)memset(&s_labels, 0, sizeof(s_labels));
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

static uint8_t mission_blast_map_matches(uint8_t blast_index)
{
    uint8_t row;
    uint8_t col;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            uint8_t boundary = (uint8_t)((row == 0u) || (col == 0u) ||
                                         (row + 1u == SOKOBAN_MAP_HEIGHT) ||
                                         (col + 1u == SOKOBAN_MAP_WIDTH));
            if(boundary && (s_initial_map[row][col] == '#') &&
               (s_current_map[row][col] != '#')) return 0u;
            if(!boundary && (s_initial_map[row][col] == '#'))
            {
                uint8_t blast_row = s_task_solution.blast_rows[blast_index];
                uint8_t blast_col = s_task_solution.blast_cols[blast_index];
                uint8_t dr = (uint8_t)(row > blast_row ?
                                       row - blast_row : blast_row - row);
                uint8_t dc = (uint8_t)(col > blast_col ?
                                       col - blast_col : blast_col - col);
                if((dr <= 1u) && (dc <= 1u) &&
                   (s_current_map[row][col] == '#')) return 0u;
            }
        }
    }
    return 1u;
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
            else if(path_executor_start(&s_task_solution))
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
                uint16_t end;
                s_blast_event_index = 0u;
                end = (s_task_solution.blast_count != 0u) ?
                      (uint16_t)(s_task_solution.blast_move_indices[0] + 1u) :
                      s_task_solution.move_count;
                if(end == 0u)
                {
                    mission_set_state(MISSION_WAIT_MAP_CLEAR);
                }
                else if(mission_start_segment(&s_task_solution, 0u, end))
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
            if(path_executor_is_done())
            {
                s_segment_running = 0u;
                s_clear_frame_count = 0u;
                if(s_blast_event_index < s_task_solution.blast_count)
                {
                    s_blast_match_count = 0u;
                    mission_set_state(MISSION_WAIT_BLAST_MAP);
                }
                else
                {
                    mission_set_state(MISSION_WAIT_MAP_CLEAR);
                }
            }
            break;

        case MISSION_WAIT_BLAST_MAP:
            if(s_frame_updated != 0u)
            {
                if(mission_blast_map_matches(s_blast_event_index))
                {
                    if(s_blast_match_count < MISSION_BLAST_STABLE_FRAMES)
                    {
                        s_blast_match_count++;
                    }
                }
                else
                {
                    s_blast_match_count = 0u;
                }
            }
            if(s_blast_match_count >= MISSION_BLAST_STABLE_FRAMES)
            {
                uint16_t begin = (uint16_t)(
                    s_task_solution.blast_move_indices[s_blast_event_index] +
                    1u);
                uint16_t end;
                s_blast_event_index++;
                end = (s_blast_event_index < s_task_solution.blast_count) ?
                      (uint16_t)(s_task_solution.blast_move_indices[
                          s_blast_event_index] + 1u) :
                      s_task_solution.move_count;
                if(begin == s_task_solution.move_count)
                {
                    mission_set_state(MISSION_WAIT_MAP_CLEAR);
                }
                else if(mission_start_segment(&s_task_solution, begin, end))
                {
                    s_segment_running = 1u;
                    mission_set_state(MISSION_EXECUTE_PUSH);
                }
                else
                {
                    mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
                }
            }
            else if(s_state_elapsed_10ms >= MISSION_RENDER_TIMEOUT_10MS)
            {
                mission_abort_level(SOKOBAN_STATUS_INVALID_MAP);
            }
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
            else if(s_state_elapsed_10ms >= MISSION_RENDER_TIMEOUT_10MS)
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
    return s_task_solution.blast_count;
}
