#include "state_machine.h"
#include <stdbool.h>
#include <stdio.h>

#define STATE_TRANS_QUEUE_SIZE 16

/* Internal State of Vehicle */
static state_t cerberus_state;

/* State Transition Map */
// TODO: fill out state transition map
static const bool valid_trans_to_from[MAX_FUNC_STATES][MAX_FUNC_STATES] = {
    /*BOOT  READY   DRIVING FAULTED*/
    {true}};

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
    return cerberus_state;
}

void vStateMachineDirector(void *pv_params)
{
    cerberus_state = IDLE_START;

    state_trans_queue = osMessageQueueNew(STATE_TRANS_QUEUE_SIZE, sizeof(state_t), NULL);

    state_t new_state;

    for (;;)
    {
        if (osOK != osMessageQueueGet(state_trans_queue, &new_state, NULL, 50))
        {
            // TODO queue fault, low criticality
            continue;
        }

        if (!valid_trans_to_from[new_state][cerberus_state])
        {
            // TODO queue fault, low criticality
            continue;
        }

        // transition state via LUT ?
        cerberus_state = new_state;
    }
}
