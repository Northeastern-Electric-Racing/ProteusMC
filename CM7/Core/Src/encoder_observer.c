#include "encoder_observer.h"

#include <assert.h>

#include "cmsis_os2.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"

#define FLAG_POLL_TIMER_INT    (0x1)
#define POLL_TIMER_INTERVAL_US (500)    /* 500us interval */
#define POLL_TIMER_PSC         (64 - 1) /* scales 64MHz down to 1MHz (assuming APB1 clock is 64MHz) */

static osThreadId_t encoder_read_thread_id;
static const osThreadAttr_t encoder_read_thread_attrs = {
    .name =       "Encoder Observer",
    .stack_size = 1024,
    .priority =   osPriorityAboveNormal3,
};
static osMessageQueueId_t encoder_data_queue;

static encoder_t *left_encoder;
static encoder_t *right_encoder;

// This function is meant to be called from the global ISR for whichever timer is used for polling
void encoder_observer_timer_notify()
{
    // set flag to notify 
    osThreadFlagsSet(encoder_read_thread_id, FLAG_POLL_TIMER_INT);
}

static void encoder_read_thread_entry(void *param)
{
    (void) param;
    float poll_interval_s = POLL_TIMER_INTERVAL_US / 1000000.0f;
    encoder_data_t data;

    while (1) {
        // wait for poll flag
        osThreadFlagsWait(FLAG_POLL_TIMER_INT, osFlagsWaitAll, osWaitForever);

        // poll timer interrupt occurred, calculate left and right encoder velocities
        data.left_velocity =  encoder_calculate_velocity(left_encoder,  poll_interval_s);
        data.right_velocity = encoder_calculate_velocity(right_encoder, poll_interval_s);

        // read absolute angle from ssi interface
        data.left_angle =  encoder_read_angle(left_encoder);
        data.right_angle = encoder_read_angle(right_encoder);

        // queue data
        osMessageQueuePut(encoder_data_queue, &data, 0, 0);
    }
}

void encoder_observer_init(encoder_t *left_enc, encoder_t *right_enc, TIM_HandleTypeDef *poll_timer_handle)
{
    assert(left_enc);
    assert(right_enc);
    assert(poll_timer_handle);

    encoder_read_thread_id = osThreadNew(encoder_read_thread_entry, NULL, &encoder_read_thread_attrs);
    assert(encoder_read_thread_id);

    left_encoder = left_enc;
    right_encoder = right_enc;

    // configure timer to increment once per microsecond, then set period
    poll_timer_handle->Instance->PSC = POLL_TIMER_PSC;
    poll_timer_handle->Instance->ARR = POLL_TIMER_INTERVAL_US - 1;

    // start poll timer in interrupt mode
    HAL_TIM_Base_Start_IT(poll_timer_handle);
}

osStatus_t encoder_observer_dequeue_data(encoder_data_t *data, uint32_t timeout)
{
    return osMessageQueueGet(encoder_data_queue, data, NULL, timeout);
}