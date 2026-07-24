#ifndef _test_h
#define _test_h

#include "zf_common_headfile.h"
#include "sokoban_solver.h"

#define POSITION_STEP_TEST_MODE (0u)
#define GYRO_SPEED_TUNE_MODE    (0u)
#define WHEEL_SPEED_TUNE_MODE   (0u)

#if ((POSITION_STEP_TEST_MODE + GYRO_SPEED_TUNE_MODE + WHEEL_SPEED_TUNE_MODE) > 1u)
#error "Only one control test mode may be enabled"
#endif

void sokoban_debug_once(void);
void label_uart_request_test_once(void);
void gyro_speed_tune_init(void);
void gyro_speed_tune_process_keys(void);
void gyro_speed_tune_update_10ms(void);
void gyro_speed_tune_screen_init(void);
void gyro_speed_tune_screen_update(void);
void wheel_speed_tune_init(void);
void wheel_speed_tune_process_keys(void);
void wheel_speed_tune_update_10ms(void);
void wheel_speed_tune_screen_init(void);
void wheel_speed_tune_screen_update(void);
void position_step_test_init(void);
void position_step_test_process_keys(void);
void position_step_test_update_10ms(void);
void position_step_test_screen_init(void);
void position_step_test_screen_update(void);

#endif
