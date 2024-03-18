#include "gatedriver.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define  PERIOD_VALUE		(uint32_t)(2000 - 1)

static void gatedrv_ready_cb(gatedriver_t* drv)
{

}

static void gatedrv_reset_cb(gatedriver_t* drv)
{

}

static void gatedrv_fault_cb(gatedriver_t* drv)
{

}

gatedriver_t* gatedrv_init(TIM_HandleTypeDef* tim, ADC_HandleTypeDef *hdma_adc, SPI_HandleTypeDef *adc_spi)
{
	/* Assert hardware params */
	assert(tim);
	assert(hdma_adc);
	assert(adc_spi);

	/* Create MPU struct */
	gatedriver_t* gatedriver = malloc(sizeof(gatedriver_t));
	assert(gatedriver);

	/* Set interfaces */
	gatedriver->tim			= tim;
	gatedriver->hdma_adc	= hdma_adc;
	gatedriver->adc_spi		= adc_spi;

	/* Init hardware */
	tim->Init.Prescaler			= 0;
	tim->Init.Period			= PERIOD_VALUE;
	tim->Init.ClockDivision		= 0;
	tim->Init.CounterMode		= TIM_COUNTERMODE_UP;
	tim->Init.RepetitionCounter	= 0;
	assert(HAL_TIM_PWM_Init(tim) != HAL_OK);

	/* Common configuration for all PWM channels */
	gatedriver->pwm_cfg.OCMode       = TIM_OCMODE_PWM1;
	gatedriver->pwm_cfg.OCPolarity   = TIM_OCPOLARITY_HIGH;
	gatedriver->pwm_cfg.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	gatedriver->pwm_cfg.OCIdleState  = TIM_OCIDLESTATE_SET;
	gatedriver->pwm_cfg.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	gatedriver->pwm_cfg.OCFastMode   = TIM_OCFAST_DISABLE;

	/* Configure DMA */
	assert(HAL_ADC_Start_DMA(gatedriver->hdma_adc, gatedriver->intern_adc_buffer, GATEDRV_SIZE_OF_ADC_DMA));

	/* Create Mutexes */
	gatedriver->tim_mutex = osMutexNew(&gatedriver->tim_mutex_attr);
	assert(gatedriver->tim_mutex);

	gatedriver->ext_adc_mutex = osMutexNew(&gatedriver->ext_adc_mutex_attr);
	assert(gatedriver->ext_adc_mutex);

	return gatedriver;
}

void gatedrv_get_phase_currents(gatedriver_t* drv, int16_t current_buf[GATEDRV_NUM_PHASES])
{
	current_buf[GATEDRV_PHASE_U] = drv->intern_adc_buffer[GATEDRV_PHASE_U];
	current_buf[GATEDRV_PHASE_V] = drv->intern_adc_buffer[GATEDRV_PHASE_V];
	current_buf[GATEDRV_PHASE_W] = drv->intern_adc_buffer[GATEDRV_PHASE_W];
}

int16_t gatedrv_read_dc_voltage(gatedriver_t* drv)
{

}

int16_t gatedrv_read_dc_current(gatedriver_t* drv)
{

}

/* Note: This has to atomically write to ALL PWM registers */
int16_t gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[GATEDRV_NUM_PHASES])
{
	/* Acquiring mutex lock */
	osStatus_t mut_stat = osMutexAcquire(drv->tim_mutex, osWaitForever);
	if (mut_stat) {
		return mut_stat;
	}

	/* Computing pulses */
	uint32_t pulses[3];
	pulses[0] = (uint32_t) (duty_cycles[0] * PERIOD_VALUE / 100);
	pulses[1] = (uint32_t) (duty_cycles[1] * PERIOD_VALUE / 100);
	pulses[2] = (uint32_t) (duty_cycles[2] * PERIOD_VALUE / 100);

	/* Getting PWM channel config */
	TIM_OC_InitTypeDef config = drv->pwm_cfg;

	/* Attempting to set channel 1 */
	config.Pulse = pulses[0];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Do nothing and return */
		return 1;
	}

	/* Attempting to set channel 2 */
	config.Pulse = pulses[1];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_2) != HAL_OK)
	{
		/* Attempt to revert last channel change and return */
		config.Pulse = drv->pulses[0];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_1);

		return 1;
	}

	/* Attempting to set channel 3 */
	config.Pulse = pulses[2];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_3) != HAL_OK)
	{
		/* Attempt to revert previous channel changes and return */
		config.Pulse = drv->pulses[0];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_1);

		config.Pulse = drv->pulses[1];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &config, TIM_CHANNEL_2);

		return 1;
	}

	/* Successful PWM modifications - save configuration, release mutex, and return */
	drv->pulses[0] = pulses[0];
	drv->pulses[1] = pulses[1];
	drv->pulses[2] = pulses[2];

	osMutexRelease(drv->tim_mutex);
	return 0;
}

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv)
{

}
