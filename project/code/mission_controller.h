#ifndef _MISSION_CONTROLLER_H_
#define _MISSION_CONTROLLER_H_

#include "sokoban_solver.h"

typedef enum
{
    MISSION_ENTER_START_ZONE = 0,
    MISSION_WAIT_VALID_MAP,
    MISSION_PLAN_INSPECTION,
    MISSION_EXECUTE_INSPECTION,
    MISSION_WAIT_TURN,
    MISSION_WAIT_SCAN_RESULT,
    MISSION_PLAN_LABELED_PUSH,
    MISSION_EXECUTE_PUSH,
    MISSION_WAIT_BLAST_MAP,
    MISSION_WAIT_MAP_CLEAR,
    MISSION_PLAN_RETURN_ENTRY,
    MISSION_EXECUTE_RETURN_ENTRY,
    MISSION_RETURN_PHYSICAL_START,
    MISSION_FINISHED,
    MISSION_FAULT
} mission_state_t;

void mission_controller_init(void);
void mission_controller_process(void);
void mission_controller_update_10ms(void);
void mission_controller_complete_turn(void);
uint8_t mission_controller_set_label(sokoban_object_type_t type, uint8_t object_index, uint8_t label);
mission_state_t mission_controller_get_state(void);
uint32_t mission_controller_get_elapsed_10ms(void);
sokoban_status_t mission_controller_get_last_status(void);
const sokoban_inspection_event_t *mission_controller_get_pending_inspection(void);
void mission_turn_process(void);
void mission_label_process(void);

uint8_t mission_controller_get_blast_event_index(void);
uint8_t mission_controller_get_blast_count(void);

extern uint8_t  label;

#endif
