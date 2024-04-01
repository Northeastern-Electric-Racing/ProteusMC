#include "state_machine.h"
#include <stdbool.h>
#include <stdio.h>

#define STATE_TRANS_QUEUE_SIZE 16

/* Internal State of Vehicle */
state_director_t proteus_state;

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

osThreadId_t sm_director_handle;
const osThreadAttr_t sm_director_attributes = {
    .name = "State Machine Director",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal3,
};

static osMessageQueueId_t state_trans_queue;

int queue_state(state_t new_state)
{
    if (!state_trans_queue)
        return 1;

    return osMessageQueuePut(state_trans_queue, &new_state, 0U, 0U);
}

state_t get_state()
{
    state_t ret;
    osStatus_t mut_stat = osMutexAcquire(proteus_state.state_mutex, osWaitForever);
	if (mut_stat) {
		return -1;
	}

    ret = proteus_state.current_state;

    osMutexRelease(proteus_state.state_mutex);

    return proteus_state.current_state;
}

void vStateMachineDirector(void *pv_params)
{
    proteus_state.current_state = LV_BOOT;

    proteus_state.state_mutex = osMutexNew(&proteus_state.state_mutex_attr);
	assert(proteus_state.state_mutex);

    state_trans_queue = osMessageQueueNew(STATE_TRANS_QUEUE_SIZE, sizeof(state_t), NULL);

    state_t new_state;

    for (;;)
    {
        if (osOK != osMessageQueueGet(state_trans_queue, &new_state, NULL, 50))
        {
            // TODO queue fault, low criticality
            continue;
        }

        //if (!valid_trans_to_from[new_state][cerberus_state])
        //{
        //    // TODO queue fault, low criticality
        //    continue;
        //}
//
        //// transition state via LUT ?
        //cerberus_state = new_state;
    }
}
