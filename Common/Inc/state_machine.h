#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "cmsis_os.h"

// TODO: add actual list of states
typedef enum {
    LV_BOOT,
    CHARGE_BOOT_CAP,
    OFFSET_CALIB,
    CLEAR,
    START,      /* Open Loop Startup */
    START_RUN,
    RUN,        /* Closed Loop Control */
    STOP_IDLE,
    STOP_NOW,
    FAULTED,
    MAX_FUNC_STATES
} state_t;

typedef struct {
    state_t current_state;
    osMutexId_t* state_mutex;
    osMutexAttr_t state_mutex_attr;
} state_director_t;

extern osThreadId_t sm_director_handle;
extern const osThreadAttr_t sm_director_attributes;

void vStateMachineDirector(void *pv_params);

/* Adds a functional state transition to be processed */
int queue_state(state_t new_state);

/* Retrieves the current functional state */
state_t get_state();

#endif