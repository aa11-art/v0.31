#ifndef _camera_sokoban_h_
#define _camera_sokoban_h_

#include "sokoban_solver.h"

/*
 * 0: normal camera mode
 * 1: fixed test map mode
 */
#define CAMERA_SOKOBAN_USE_FIXED_TEST_MAP   (0u)

void camera_sokoban_init(void);
void camera_sokoban_process(void);
uint8_t camera_sokoban_take_last_solution(sokoban_solution_t *out);

const sokoban_solution_t *camera_sokoban_get_last_solution(void);
sokoban_status_t camera_sokoban_get_last_status(void);
uint8_t camera_sokoban_copy_latest_frame(
    char solver_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t *player_row,
    uint8_t *player_col,
    uint8_t *map_all_zero,
    uint32_t *sequence);

#endif
