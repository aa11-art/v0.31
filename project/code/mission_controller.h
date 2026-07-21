#ifndef _MISSION_CONTROLLER_H_
#define _MISSION_CONTROLLER_H_

#include "sokoban_solver.h"

#ifndef MISSION_FIRST_LEVEL
#define MISSION_FIRST_LEVEL (1u)
#endif

#ifndef MISSION_LAST_LEVEL
#define MISSION_LAST_LEVEL  (3u)
#endif

#if (MISSION_FIRST_LEVEL < 1u) || (MISSION_FIRST_LEVEL > 3u)
#error "MISSION_FIRST_LEVEL must be 1, 2 or 3"
#endif

#if (MISSION_LAST_LEVEL < 1u) || (MISSION_LAST_LEVEL > 3u)
#error "MISSION_LAST_LEVEL must be 1, 2 or 3"
#endif

#if MISSION_FIRST_LEVEL > MISSION_LAST_LEVEL
#error "MISSION_FIRST_LEVEL must not exceed MISSION_LAST_LEVEL"
#endif

typedef enum
{
    MISSION_BOOT_DELAY = 0,
    MISSION_LEAVE_START_ZONE,
    MISSION_WAIT_VALID_MAP,
    MISSION_PLAN_PLAIN_PUSH,
    MISSION_PLAN_INSPECTION,
    MISSION_EXECUTE_INSPECTION,
    MISSION_WAIT_TURN,
    MISSION_WAIT_SCAN_RESULT,
    MISSION_PLAN_LABELED_PUSH,
    MISSION_EXECUTE_PUSH,
    MISSION_WAIT_PUSH_SETTLE,
    MISSION_WAIT_PUSH_MAP,
    MISSION_PLAN_NORMAL_RETURN_ENTRY,
    MISSION_EXECUTE_NORMAL_RETURN_ENTRY,
    MISSION_NORMAL_RETURN_PHYSICAL_START,
    MISSION_PLAN_ABORT_RETURN_ENTRY,
    MISSION_EXECUTE_ABORT_RETURN_ENTRY,
    MISSION_ABORT_RETURN_PHYSICAL_START,
    MISSION_ABORT_HOLD,
    MISSION_LEVEL_CHANGE,
    MISSION_FINISHED,
    MISSION_FAULT
} mission_state_t;

typedef enum
{
    MISSION_LEVEL_RESULT_NONE = 0,
    MISSION_LEVEL_RESULT_SUCCESS,
    MISSION_LEVEL_RESULT_ABORTED
} mission_level_result_t;

typedef enum
{
    MISSION_FATAL_NONE = 0,
    MISSION_FATAL_PATH,
    MISSION_FATAL_RETURN_PATH,
    MISSION_FATAL_EMERGENCY_STOP
} mission_fatal_fault_t;

typedef enum
{
    MISSION_RECOVERY_NONE = 0,
    MISSION_RECOVERY_EXACT,
    MISSION_RECOVERY_PLAYER,
    MISSION_RECOVERY_BOX,
    MISSION_RECOVERY_ACTUAL_MAP,
    MISSION_RECOVERY_REJECTED
} mission_recovery_status_t;

void mission_controller_init(void);
void mission_controller_process(void);
void mission_controller_update_10ms(void);
void mission_controller_complete_turn(void);
void mission_controller_emergency_stop(void);
uint8_t mission_controller_set_label(sokoban_object_type_t type, uint8_t object_index, uint8_t label);
mission_state_t mission_controller_get_state(void);
uint32_t mission_controller_get_elapsed_10ms(void);
sokoban_status_t mission_controller_get_last_status(void);
uint8_t mission_controller_get_level(void);
mission_level_result_t mission_controller_get_level_result(void);
mission_level_result_t mission_controller_get_last_level_result(void);
mission_fatal_fault_t mission_controller_get_fatal_fault(void);
uint16_t mission_controller_get_abort_hold_remaining_10ms(void);
uint8_t mission_controller_get_map_stable_frames(void);
uint16_t mission_controller_get_push_checkpoint_index(void);
uint16_t mission_controller_get_push_checkpoint_total(void);
uint8_t mission_controller_get_recovery_count(void);
mission_recovery_status_t mission_controller_get_recovery_status(void);
uint8_t mission_controller_should_stop(void);
const sokoban_inspection_event_t *mission_controller_get_pending_inspection(void);
void mission_turn_process(void);
void mission_label_process(void);

uint8_t mission_controller_get_blast_event_index(void);
uint8_t mission_controller_get_blast_count(void);

extern uint8_t  label;

#endif
