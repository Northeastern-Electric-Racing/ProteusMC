#ifndef INC_ENCODER_H
#define INC_ENCODER_H

#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"


typedef struct inc_encoder {
    TIM_HandleTypeDef *tim;
    uint16_t z_pin_num;
} inc_encoder_t;

/**
 * @brief Creates new incremental encoder operating on given timer peripheral
 * and z pulse pin number
 * 
 * @param timer_handle 
 * @param z_pin_num Pin number for z pulse signal, must have been configured as
 * EXTI through CubeMX
 * @return inc_encoder_t* pointer to new incremental encoder object
 */
inc_encoder_t *inc_encoder_init(TIM_HandleTypeDef *timer_handle, uint16_t z_pin_num);

/**
 * @brief Reads estimated absolute angle from encoder and converts it to radians
 * 
 * @param encoder 
 * @param angle pointer to angle in radians
 * @return int16_t nonzero if the read fails, 0 on success
 */
int16_t inc_encoder_get_angle(inc_encoder_t *encoder, float *angle);

#endif /* INC_ENCODER_H */