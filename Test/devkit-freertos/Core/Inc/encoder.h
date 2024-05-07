#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f3xx.h"
#include "stm32f3xx_hal_tim.h"

#include "foc_ctrl.h"

typedef struct encoder {
    TIM_HandleTypeDef *enc_timer;

    uint32_t prev_cnt_value;
} encoder_t;

void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *enc_timer_handle);

/**
 * @brief Notifies encoder that poll interval has elapsed, sends speed and position data to foc controller
 * NOTE: This function is meant to be called from the global ISR for whichever timer is used for polling
 * 
 * @param encoder 
 */
void encoder_poll_interval_notify(encoder_t *encoder);

/**
 * @brief Resets the encoder's counter in response to z pulse occurring
 * 
 * @param encoder 
 */
void encoder_z_pulse_handler(encoder_t *encoder);

/**
 * @brief Reads angle from encoder
 * 
 * @param encoder 
 * @return float angle (rad)
 */
float encoder_read_angle(encoder_t *encoder);

/**
 * @brief Calculates angular velocity from encoder given dt
 * 
 * @param encoder 
 * @param dt delta time (s)
 * @return float angular velocity (rad/s)
 */
float encoder_calculate_velocity(encoder_t *encoder, float dt);

#endif /* ENCODER_H */
