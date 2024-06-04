#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "cmsis_os.h"
#include "proteus_config.h"

typedef struct {
    state_t current_state;
    osMutexId_t* state_mutex;
    osMutexAttr_t state_mutex_attr;
    osMessageQueueId_t state_trans_queue;
    osThreadId_t thread;
} state_machine_t;

extern const osThreadAttr_t sm_director_attributes;

/* Initialize a State Machine */
void state_machine_init(state_machine_t *sm);

/* Adds a functional state transition to be processed */
int state_machine_queue_state(state_machine_t *sm, state_t new_state);

/* Retrieves the current functional state */
state_t state_machine_get_state(state_machine_t *sm);

void vStateMachineDirector(void *pv_params);

#endif