#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "cmsis_os.h"

// TODO: add actual list of states
typedef enum
{
    IDLE_START,
    CHARGE_BOOT_CAP,
    OFFSET_CALIB,
    CLEAR,
    START,
    SWITCH_OVER,
    START_RUN,
    RUN,
    ANY_STOP,
    STOP_IDLE,
    FAULT_NOW,
    MAX_FUNC_STATES
} state_t;

extern osThreadId_t sm_director_handle;
extern const osThreadAttr_t sm_director_attributes;

void vStateMachineDirector(void *pv_params);

/* Adds a functional state transition to be processed */
int queue_state(state_t new_state);

/* Retrieves the current functional state */
state_t get_state();

#endif