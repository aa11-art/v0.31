#ifndef _sokoban_solver_h_
#define _sokoban_solver_h_


#include "zf_common_headfile.h"

#define SOKOBAN_MAP_HEIGHT      (12u)
#define SOKOBAN_MAP_WIDTH       (16u)
#define SOKOBAN_MAP_STRIDE      (SOKOBAN_MAP_WIDTH + 1u)
#define SOKOBAN_MAX_BOXES       (10u)
#define SOKOBAN_MAX_BOMBS       (3u)
#define SOKOBAN_MAX_MOVES       (2048u)
#define SOKOBAN_PUSH_FLAG_BYTES ((SOKOBAN_MAX_MOVES + 7u) / 8u)
#define SOKOBAN_MAX_OBJECTS     (SOKOBAN_MAX_BOXES * 2u)
#define SOKOBAN_BOMB_LABEL      (255u)
#define SOKOBAN_INVALID_CELL    (0xFFu)
#define SOKOBAN_INVALID_MOVE_INDEX (0xFFFFu)

typedef enum
{
    SOKOBAN_STATUS_OK = 0,
    SOKOBAN_STATUS_INVALID_ARGUMENT,
    SOKOBAN_STATUS_INVALID_MAP,
    SOKOBAN_STATUS_BOX_GOAL_MISMATCH,
    SOKOBAN_STATUS_TOO_MANY_BOXES,
    SOKOBAN_STATUS_NO_MEMORY,
    SOKOBAN_STATUS_PATH_OVERFLOW,
    SOKOBAN_STATUS_UNSOLVABLE,
    SOKOBAN_STATUS_INVALID_LABELS,
    SOKOBAN_STATUS_INSPECTION_UNREACHABLE
} sokoban_status_t;

typedef enum
{
    SOKOBAN_DIR_UP = 0,
    SOKOBAN_DIR_RIGHT,
    SOKOBAN_DIR_DOWN,
    SOKOBAN_DIR_LEFT
} sokoban_direction_t;

typedef enum
{
    SOKOBAN_BODY_FORWARD = 0,
    SOKOBAN_BODY_RIGHT,
    SOKOBAN_BODY_BACKWARD,
    SOKOBAN_BODY_LEFT
} sokoban_body_direction_t;

typedef enum
{
    SOKOBAN_OBJECT_BOX = 0,
    SOKOBAN_OBJECT_GOAL
} sokoban_object_type_t;

typedef struct
{
    uint8_t solved;
    uint16_t move_count;
    uint16_t push_count;
    uint16_t expanded_forward;
    uint16_t expanded_reverse;
    uint8_t blast_count;
    uint16_t blast_move_indices[SOKOBAN_MAX_BOMBS];
    uint8_t blast_rows[SOKOBAN_MAX_BOMBS];
    uint8_t blast_cols[SOKOBAN_MAX_BOMBS];
    uint8_t push_flags[SOKOBAN_PUSH_FLAG_BYTES];
    char move_seq[SOKOBAN_MAX_MOVES + 1u];
} sokoban_solution_t;

typedef struct
{
    uint16_t move_index;
    uint8_t object_index;
    uint8_t object_type;
    uint8_t object_row;
    uint8_t object_col;
    uint8_t stand_row;
    uint8_t stand_col;
    sokoban_direction_t face_direction;
    int8_t quarter_turns;
} sokoban_inspection_event_t;

typedef struct
{
    sokoban_solution_t route;
    uint8_t event_count;
    uint8_t final_row;
    uint8_t final_col;
    sokoban_direction_t final_heading;
    sokoban_inspection_event_t events[SOKOBAN_MAX_OBJECTS];
} sokoban_inspection_plan_t;

typedef struct
{
    uint8_t box_count;
    uint8_t goal_count;
    uint8_t box_labels[SOKOBAN_MAX_BOXES];
    uint8_t goal_labels[SOKOBAN_MAX_BOXES];
} sokoban_label_table_t;

typedef struct
{
    uint8_t found;
    uint8_t wall_row;
    uint8_t wall_col;
    uint16_t path_len_after_blast;
} sokoban_bomb_plan_t;

/*
 * Map tokens:
 * '#' = wall
 * '-' = floor
 * '.' = goal
 * '$' = box
 * '*' = bomb box (supported by labeled mission solver only)
 * '@' = player
 * '+' = player on goal
 *
 * Preferred solver: decomposed greedy single-box BFS.
 * Legacy fallback: sokoban_solve_bidirectional_astar() uses single-direction A*
 * and requires box_count == goal_count.
 * Rule: when a box enters a goal, both disappear immediately.
 * solution->move_seq returns U/D/L/R.
 * Use sokoban_moves_to_chinese() if you need "up/down/left/right" in Chinese.
 */
sokoban_status_t sokoban_solve_decomposed(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                          sokoban_solution_t *solution);
sokoban_status_t sokoban_solve_bidirectional_astar(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                                   sokoban_solution_t *solution);
sokoban_status_t sokoban_plan_walk(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                   uint8_t start_row,
                                   uint8_t start_col,
                                   uint8_t target_row,
                                   uint8_t target_col,
                                   sokoban_solution_t *solution);
sokoban_status_t sokoban_find_best_bomb_wall(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                             uint8_t start_row,
                                             uint8_t start_col,
                                             uint8_t target_row,
                                             uint8_t target_col,
                                             sokoban_bomb_plan_t *plan);
sokoban_direction_t sokoban_direction_turn(sokoban_direction_t heading, int8_t quarter_turns);
sokoban_body_direction_t sokoban_world_to_body(sokoban_direction_t heading,
                                               sokoban_direction_t world_direction);
uint8_t sokoban_solution_move_is_push(const sokoban_solution_t *solution,
                                      uint16_t move_index);
sokoban_status_t sokoban_plan_inspection(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                         sokoban_direction_t initial_heading,
                                         sokoban_inspection_plan_t *plan);
sokoban_status_t sokoban_solve_labeled(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                       uint8_t start_row,
                                       uint8_t start_col,
                                       const sokoban_label_table_t *labels,
                                       sokoban_solution_t *solution);
const char *sokoban_move_to_chinese(char move);
uint16_t sokoban_moves_to_chinese(const char *moves, uint16_t move_count, char *out, uint16_t out_capacity);
const char *sokoban_status_string(sokoban_status_t status);

#endif
