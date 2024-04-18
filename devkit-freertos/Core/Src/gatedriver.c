#include "gatedriver.h"
#include "stm32f3xx.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

// #define  PERIOD_VALUE		(uint32_t)(2000 - 1)
// from MCSDK
#define  PERIOD_VALUE		((PWM_PERIOD_CYCLES) / 2)

gatedriver_t* gatedrv_init(TIM_HandleTypeDef* tim, ADC_HandleTypeDef *phase_adc)
{
	/* Assert hardware params */
	assert(tim);
	assert(phase_adc);
	// assert(adc_spi);

	/* Create MPU struct */
	gatedriver_t* gatedriver = malloc(sizeof(gatedriver_t));
	assert(gatedriver);

	/* Set interfaces */
	gatedriver->tim			= tim;
	gatedriver->phase_adc	= phase_adc;
	// gatedriver->adc_spi		= adc_spi;

	/* Common configuration for all PWM channels */
	TIM_OC_InitTypeDef channel_config = {
		.Pulse        = 0,
		.OCMode       = TIM_OCMODE_PWM1,
		.OCPolarity   = TIM_OCPOLARITY_HIGH,
		.OCNPolarity  = TIM_OCNPOLARITY_HIGH,
		.OCIdleState  = TIM_OCIDLESTATE_RESET,
		.OCNIdleState = TIM_OCNIDLESTATE_RESET,
		.OCFastMode   = TIM_OCFAST_DISABLE,
	};

	// start ADC timing pwm generation
	assert(HAL_TIM_PWM_Start(tim, TIM_CHANNEL_4) == HAL_OK);

	// enable Phase reading ADC in interrupt mode
	assert(HAL_ADCEx_InjectedStart_IT(phase_adc) == HAL_OK);

	/* Attempting to set channel 1 */
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_1) == HAL_OK);
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_2) == HAL_OK);
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_3) == HAL_OK);

	HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_3);

	return gatedriver;
}

/* Note: This has to atomically write to ALL PWM registers */
int16_t gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[GATEDRV_NUM_PHASES])
{
	drv->tim->Instance->CCR1 = (uint32_t) (duty_cycles[0] * PERIOD_VALUE);
	drv->tim->Instance->CCR2 = (uint32_t) (duty_cycles[1] * PERIOD_VALUE);
	drv->tim->Instance->CCR3 = (uint32_t) (duty_cycles[2] * PERIOD_VALUE);

	return 0;
}
