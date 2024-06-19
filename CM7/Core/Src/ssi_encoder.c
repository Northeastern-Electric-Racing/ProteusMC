#include "ssi_encoder.h"

#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"

#define ENCODER_MAX_COUNT (1 << 13)

ssi_encoder_t *ssi_encoder_init(SPI_HandleTypeDef *spi_handle)
{
	// make sure the handle exists
	assert(spi_handle);

	// allocate new struct
	ssi_encoder_t *encoder = malloc(sizeof(ssi_encoder_t));
	assert(encoder);

	encoder->hspi = spi_handle;

	return encoder;
}

int16_t ssi_encoder_get_angle(ssi_encoder_t *encoder, float *angle)
{
	uint16_t counts = 0;

	if (HAL_SPI_Receive(encoder->hspi, (uint8_t *)&counts, 1, 10) != HAL_OK)
		return 1;

	// convert counts to radians
	*angle = (((float)counts) / ENCODER_MAX_COUNT) * 2 * M_PI;

	return 0;
}