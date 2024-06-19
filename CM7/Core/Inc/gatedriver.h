#ifndef GATEDRIVER_H
#define GATEDRIVER_H

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "foc_ctrl.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Note that these phases readings should ALWAYS be mapped to the corresponding indices
 * Ensure the ADC DMA is mapped the same across boards
 */
enum { GATEDRV_PHASE_U, GATEDRV_PHASE_V, GATEDRV_PHASE_W, GATEDRV_NUM_PHASES };

enum {
	GATEDRV_DC_CURRENT =
		GATEDRV_NUM_PHASES, /* Keep index rolling from phase enum */
	GATEDRV_IGBT_TEMP,
	GATEDRV_SIZE_OF_ADC_DMA
};

/* Definition of gatedriver struct */
typedef struct {
	TIM_HandleTypeDef *tim;
	osMutexId_t *tim_mutex;
	TIM_OC_InitTypeDef pwm_cfg;
	uint32_t pulses[GATEDRV_NUM_PHASES];

	ADC_HandleTypeDef *phase_adc;
	SPI_HandleTypeDef *adc_spi;
	uint32_t intern_adc_buffer[GATEDRV_SIZE_OF_ADC_DMA];

	osMutexId_t *tim_mutex_mutex;
	osMutexAttr_t tim_mutex_attr;
	osMutexId_t *ext_adc_mutex;
	osMutexAttr_t ext_adc_mutex_attr;

	osThreadId_t write_thread;
	foc_ctrl_t *controller;

	int initial_reading_taken;
	float channel_offsets[3];
} gatedriver_t;

extern const osThreadAttr_t phase_actor_attributes;
void vPhaseActor(void *pv_params);

/* initialize a new gatedriver */
void gatedrv_init(gatedriver_t *gatedriver, TIM_HandleTypeDef *tim,
		  ADC_HandleTypeDef *phase_adc, foc_ctrl_t *controller);

/* Note: This has to atomically write to ALL PWM registers */
void gatedrv_write_pwm(gatedriver_t *drv, float duty_cycles[]);

/* read the internal IGBT temp */
int16_t gatedrv_read_igbt_temp(gatedriver_t *drv);

/* Read the phase currents */
void gatedrv_get_phase_currents(gatedriver_t *drv, float phase_currents[3]);

/* read the dc voltage (V) */
int16_t gatedrv_read_dc_voltage(gatedriver_t *drv);

/* read the dc current (A) */
int16_t gatedrv_read_dc_current(gatedriver_t *drv);

/* read the internal IGBT temp */
int16_t gatedrv_read_igbt_temp(gatedriver_t *drv);

#endif /* GATEDRIVER_H */
