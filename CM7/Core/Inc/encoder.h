#ifndef ENCODER_H
#define ENCODER_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"
#include "foc_ctrl.h"
#include "cmsis_os.h"
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *hspi;
    TIM_HandleTypeDef *enc_timer;
    uint32_t prev_cnt_value;
    foc_ctrl_t *controller;
    osThreadId_t thread;
} encoder_t;

extern const osThreadAttr_t encoder_observer_attributes;

void vEncoderObserver(void *pv_params);

/**
 * Initialize Encoder
*/
void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *enc_timer_handle, SPI_HandleTypeDef *spi_handle, foc_ctrl_t * controller);

/**
 * @brief Notifies encoder that poll interval has elapsed, sends speed and position data to foc controller
 * NOTE: This function is meant to be called from the global ISR for whichever timer is used for polling
 *
 * @param encoder
 */
void encoder_poll_interval_notify(encoder_t *encoder);

#endif /* SSI_ENCODER_H */
