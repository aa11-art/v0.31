#ifndef _PATH_EXECUTOR_H_
#define _PATH_EXECUTOR_H_

#include "sokoban_solver.h"

void path_executor_init(void);
uint8_t path_executor_start(const sokoban_solution_t *solution);
uint8_t path_executor_start_with_distance(const sokoban_solution_t *solution, float step_distance);
void path_executor_update_10ms(void);
uint8_t path_executor_is_idle(void);
uint8_t path_executor_is_done(void);
void path_executor_set_heading(sokoban_direction_t heading);
float path_executor_get_last_step_distance(void);

#endif
