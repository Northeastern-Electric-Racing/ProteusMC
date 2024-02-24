#ifndef GATEDRIVER_H
#define GATEDRIVER_H

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define ADC_BUF_LEN     4

enum {
    PHASE_U,
    PHASE_V,
    PHASE_W,
    NUM_PHASES
};

typedef struct {
	TIM_HandleTypeDef* tim;
    osMutexId_t* tim_mutex;
    TIM_OC_InitTypeDef* pPWMConfig;
	uint32_t pulses[NUM_PHASES];

    DMA_HandleTypeDef *hdma_adc;
	SPI_HandleTypeDef *adc_spi;
    uint32_t intern_adc_buffer[ADC_BUF_LEN];

    osMutexId_t* tim_mutex_mutex;
    osMutexAttr_t tim_mutex_attr;
    osMutexId_t* ext_adc_mutex;
    osMutexAttr_t ext_adc_mutex_attr;
} gatedriver_t;

gatedriver_t* gatedrv_init(TIM_HandleTypeDef* tim, DMA_HandleTypeDef *hdma_adc, SPI_HandleTypeDef *adc_spi);

int16_t gatedrv_read_dc_voltage(gatedriver_t* drv);

int16_t gatedrv_read_dc_current(gatedriver_t* drv);

/* Note: This has to atomically write to ALL PWM registers */
int16_t gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[]);

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv);

#endif /* GATEDRIVER_H */
