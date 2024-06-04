#include "encoder.h"

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"

#define ENCODER_MAX_COUNT (1 << 13)
#define PULSES_PER_REV         (8191)

void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *enc_timer_handle, SPI_HandleTypeDef *spi_handle, foc_ctrl_t *controller)
{
    /* Assert Params */
    assert(spi_handle);
    assert(enc_timer_handle);
    assert(controller);

    /* Start Incremental Encoder */
    encoder->enc_timer = enc_timer_handle;
    assert(HAL_TIM_Encoder_Start(encoder->enc_timer, TIM_CHANNEL_ALL) == HAL_OK);

    encoder->controller = controller;

    /* Save SSI interface */
    encoder->hspi = spi_handle;
}

static int16_t encoder_get_angle_ssi(encoder_t *encoder, float *angle)
{
    uint16_t counts = 0;

    if (HAL_SPI_Receive(encoder->hspi, (uint8_t *) &counts, 1, 10) != HAL_OK)
        return 1;

    /* convert counts to radians */
    *angle = (((float) counts) / ENCODER_MAX_COUNT) * 2 * M_PI;

    return 0;
}

static void encoder_z_pulse_handler(encoder_t *encoder)
{
    encoder->enc_timer->Instance->CNT = 0;
}

static float encoder_get_angle_incr(encoder_t *encoder)
{
    uint16_t counts = encoder->enc_timer->Instance->CNT;

    /* convert counts to radians */
    return (((float) (counts % (PULSES_PER_REV + 1))) / PULSES_PER_REV) * 2 * M_PI;
}

static float encoder_calculate_velocity(encoder_t *encoder, float dt)
{
    /* Use timer in encoder mode to calculate velocity using previous count value */
    uint32_t current_count = encoder->enc_timer->Instance->CNT;
    int32_t count_delta = current_count - encoder->prev_cnt_value;

    /* Update previous value with current */
    encoder->prev_cnt_value = current_count;

    return ((float) count_delta / PULSES_PER_REV) * 2 * M_PI / dt;
}

const osThreadAttr_t encoder_observer_attributes = {
	.name = "Encoder Observer",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityHigh,
};

void vEncoderObserver(void *pv_params)
{
	osStatus_t status;
    float ssi_position;

    foc_data_t msg = {.type = FOCDATA_ROTOR_POSITION};

	encoder_t *encoder = (encoder_t *)pv_params;
	assert(encoder);

	for (;;)
	{
        /* Retrieve SSI position */
        if (encoder_get_angle_ssi(encoder, &ssi_position));
            continue;

        /* Fuse SSI position with incremental position */
        msg.payload.rotor_position = (ssi_position + encoder_get_angle_incr(encoder)) / 2;

        /* Enqueue reading */
        foc_queue_frame(encoder->controller, &msg);

        /* Wait */
        osDelay(10);
	}
}
