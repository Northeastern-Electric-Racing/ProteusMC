#include "state_machine.h"
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>

#define STATE_TRANS_QUEUE_SIZE 16

/* State Transition Map */
// TODO: fill out state transition map
static const bool valid_trans_to_from[MAX_FUNC_STATES][MAX_FUNC_STATES] = {
    /* IDLE_START, CHARGE_BOOT_CAP, OFFSET_CALIB, CLEAR, START, START_RUN, RUN, STOP_IDLE, STOP_NOW, FAULTED */
    {true, true, false, false, false, false, false, false, false, true},    /* IDLE_START */
    {true, true, false, false, false, false, false, false, false, true},    /* CHARGE_BOOT_CAP */
    {true, true, true, false, false, false, false, false, false, true},    /* OFFSET_CALIB */
    {true, true, false, true, false, false, false, false, false, true},    /* CLEAR */
    {true, true, false, true, true, false, false, false, false, true},    /* START */
    {true, true, false, false, true, true, false, false, false, true},    /* START_RUN */
    {true, true, false, false, false, true, true, false, false, true},    /* RUN */
    {true, true, false, false, false, false, true, true, false, true},    /* STOP_IDLE */
    {true, true, false, false, false, false, false, true, true, true},    /* STOP_NOW */
    {true, true, false, false, false, false, false, false, true, true},    /* FAULTED */
};

void state_machine_init(state_machine_t *sm)
{
    sm->current_state = LV_BOOT;

    sm->state_mutex = osMutexNew(&sm->state_mutex_attr);
	assert(sm->state_mutex);

    sm->state_trans_queue = osMessageQueueNew(STATE_TRANS_QUEUE_SIZE, sizeof(state_t), NULL);
}

int state_machine_queue_state(state_machine_t *sm, state_t new_state)
{
    if (!sm->state_trans_queue)
        return 1;

    return osMessageQueuePut(sm->state_trans_queue, &new_state, 0U, 0U);
}

state_t state_machine_get_state(state_machine_t *sm)
{
    state_t ret;
    osStatus_t mut_stat = osMutexAcquire(sm->state_mutex, osWaitForever);
	if (mut_stat) {
		return -1;
	}

    ret = sm->current_state;

    osMutexRelease(sm->state_mutex);

    return ret;
}

const osThreadAttr_t sm_director_attributes = {
    .name = "State Machine Director",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal3,
};

void vStateMachineDirector(void *pv_params)
{
    state_machine_t *sm = (state_machine_t *)pv_params;
	assert(sm);

    state_t new_state;

    for (;;)
    {
        if (osOK != osMessageQueueGet(sm->state_trans_queue, &new_state, NULL, 50))
        {
            // TODO queue fault, low criticality
            continue;
        }

        if (!valid_trans_to_from[new_state][sm->current_state])
        {
            // TODO queue fault, low criticality
            continue;
        }

        printf("Transitioning state!\r\n");
        /* transition state via LUT */
        sm->current_state = new_state;
    }
}