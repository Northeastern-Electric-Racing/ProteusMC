#ifndef SSI_ENCODER_H
#define SSI_ENCODER_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"
#include <stdint.h>

typedef struct ssi_encoder {
	SPI_HandleTypeDef *hspi;
} ssi_encoder_t;

/**
 * @brief Creates new ssi encoder operating on given spi peripheral
 * 
 * @param spi_handle 
 * @return ssi_encoder_t* pointer to new ssi encoder object
 */
ssi_encoder_t *ssi_encoder_init(SPI_HandleTypeDef *spi_handle);

/**
 * @brief Reads absolute angle from encoder and converts it to radians
 * NOTE: The datasheet for the encoder states that the minimum delay
 *       between reads must be between 12.5 us and 20.5 us
 * 
 * @param encoder 
 * @param angle pointer to angle in radians
 * @return int16_t nonzero if the read fails, 0 on success
 */
int16_t ssi_encoder_get_angle(ssi_encoder_t *encoder, float *angle);

#endif /* SSI_ENCODER_H */