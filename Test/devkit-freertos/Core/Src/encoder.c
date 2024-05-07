#include "encoder.h"

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "stm32f3xx.h"
#include "stm32f3xx_hal_tim.h"

#define PULSES_PER_REV         (8191)

void encoder_z_pulse_handler(encoder_t *encoder)
{
    encoder->enc_timer->Instance->CNT = 0;
}

void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *enc_timer_handle)
{
    // make sure params are valid
    assert(enc_timer_handle);
    assert(encoder);

    encoder->enc_timer = enc_timer_handle;

    // start encoder mode timer
    assert(HAL_TIM_Encoder_Start(encoder->enc_timer, TIM_CHANNEL_ALL) == HAL_OK);
}

float encoder_read_angle(encoder_t *encoder)
{
    uint16_t counts = encoder->enc_timer->Instance->CNT;

    // convert counts to radians
    return (((float) (counts % (PULSES_PER_REV + 1))) / PULSES_PER_REV) * 2 * M_PI;
}

float encoder_calculate_velocity(encoder_t *encoder, float dt)
{
    // use timer in encoder mode to calculate velocity using previous count value
    uint32_t current_count = encoder->enc_timer->Instance->CNT;
    int32_t count_delta = current_count - encoder->prev_cnt_value;
    // update previous value with current;
    encoder->prev_cnt_value = current_count;

    return ((float) count_delta / PULSES_PER_REV) * 2 * M_PI / dt;
}
