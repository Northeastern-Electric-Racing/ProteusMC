#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"

typedef struct encoder {
    SPI_HandleTypeDef *hspi;
    TIM_HandleTypeDef *htim;
    uint32_t prev_cnt_value;
} encoder_t;

/**
 * @brief Creates new encoder operating on given spi and timer peripheral
 * 
 * @param spi_handle 
 * @param timer_handle 
 * @return encoder_t* pointer to new ssi encoder object
 */
encoder_t *encoder_init(SPI_HandleTypeDef *spi_handle, TIM_HandleTypeDef *timer_handle);

/**
 * @brief Reads absolute angle from encoder in radians
 * 
 * @param encoder 
 * @return float angle in radians
 */
float encoder_read_angle(encoder_t *encoder);

/**
 * @brief Calculates estimated angular velocity from encoder in rad/s
 * 
 * @param encoder 
 * @param dt change in time from last call to the function
 * @return float angular velocity in radians per second
 */
float encoder_calculate_velocity(encoder_t *encoder, float dt);

#endif /* ENCODER_H */