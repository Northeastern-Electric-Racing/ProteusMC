#ifndef ENCODER_OBSERVER_H
#define ENCODER_OBSERVER_H


#include <stdint.h>
#include "encoder.h"
#include "cmsis_os2.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"

typedef struct encoder_data {
    float left_angle;
    float left_velocity;
    float right_angle;
    float right_velocity;
} encoder_data_t;

void encoder_observer_init(encoder_t *left_encoder, encoder_t *right_encoder, TIM_HandleTypeDef *poll_timer_handle);

osStatus_t encoder_observer_dequeue_data(encoder_data_t *data, uint32_t timeout);

void encoder_observer_timer_notify();

#endif /* ENCODER_OBSERVER_H */
