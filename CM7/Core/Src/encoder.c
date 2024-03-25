#include "encoder.h"

#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"

#define ENCODER_MAX_COUNT ((1 << 13) - 1)
#define PULSES_PER_REV    (2048) /* from datasheet */

encoder_t *encoder_init(SPI_HandleTypeDef *spi_handle, TIM_HandleTypeDef *timer_handle)
{
    // make sure the handle exists
    assert(spi_handle);

    // allocate new struct
    encoder_t *encoder = malloc(sizeof(encoder_t));
    assert(encoder);    

    encoder->hspi = spi_handle;
    encoder->htim = timer_handle;
    // get the current counter value so we can calculate velocity
    encoder->prev_cnt_value = timer_handle->Instance->CNT;

    return encoder;
}

float encoder_read_angle(encoder_t *encoder)
{
    uint16_t counts = 0;

    if (HAL_SPI_Receive(encoder->hspi, (uint8_t *) &counts, 1, 10) != HAL_OK)
        return 1;

    // convert counts to radians
    return (((float) counts) / ENCODER_MAX_COUNT) * 2 * M_PI;
}

float encoder_calculate_velocity(encoder_t *encoder, float dt)
{
    // use timer in encoder mode to calculate velocity using previous count value
    uint32_t current_count = encoder->htim->Instance->CNT;
    int32_t count_delta = current_count - encoder->prev_cnt_value;
    // update previous value with current;
    encoder->prev_cnt_value = current_count;

    return ((float) count_delta / PULSES_PER_REV) * 2 * M_PI / dt;
}
