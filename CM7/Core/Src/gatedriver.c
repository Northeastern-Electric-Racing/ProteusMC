#include "gatedriver.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

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

#define ADC_MAX_VALUE              (0xFFFF)
//                             (Scale ADC sample to voltage)  (Divide gain)  (V/R, R=0.33ohms)
//                                            |                    |                |
#define SAMPLE_TO_AMPS             ((3.3f / ADC_MAX_VALUE) * (1.0f / 1.53f) * (1.0f / 0.33f))

void gatedrv_init(gatedriver_t *gatedriver, TIM_HandleTypeDef* tim, ADC_HandleTypeDef *phase_adc)
{
	/* Assert hardware params */
	assert(tim);
	assert(phase_adc);

	/* Set interfaces */
	gatedriver->tim			= tim;
	gatedriver->phase_adc	= phase_adc;

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

	/* Enable phase reading ADC in interrupt mode */
	assert(HAL_ADCEx_InjectedStart_IT(gatedriver->phase_adc) == HAL_OK);
	//assert(HAL_ADC_Start_DMA(gatedriver->phase_adc, gatedriver->intern_adc_buffer, GATEDRV_SIZE_OF_ADC_DMA) == HAL_OK);

	/* Configure PWM Channels */
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_1) == HAL_OK);
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_2) == HAL_OK);
	assert(HAL_TIM_PWM_ConfigChannel(gatedriver->tim, &channel_config, TIM_CHANNEL_3) == HAL_OK);

	/* Start 3 phase PWM generation + inverted signals with deadtime */
	assert(HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_1) == HAL_OK);
	assert(HAL_TIMEx_PWMN_Start(gatedriver->tim, TIM_CHANNEL_1) == HAL_OK);
	assert(HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_2) == HAL_OK);
	assert(HAL_TIMEx_PWMN_Start(gatedriver->tim, TIM_CHANNEL_2) == HAL_OK);
	assert(HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_3) == HAL_OK);
	assert(HAL_TIMEx_PWMN_Start(gatedriver->tim, TIM_CHANNEL_3) == HAL_OK);

	/* Start Fake PWM signal for ADC timing */
	assert(HAL_TIM_PWM_Start(gatedriver->tim, TIM_CHANNEL_4) == HAL_OK);

	gatedriver->initial_reading_taken = 0;
}

/* Note: This has to atomically write to ALL PWM registers */
void gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[GATEDRV_NUM_PHASES])
{
	// Not sure why a divide by 2 is necessary here but it is :)
	drv->tim->Instance->CCR1 = (uint32_t) (duty_cycles[0] * PWM_PERIOD_CYCLES) / 2;
	drv->tim->Instance->CCR2 = (uint32_t) (duty_cycles[1] * PWM_PERIOD_CYCLES) / 2;
	drv->tim->Instance->CCR3 = (uint32_t) (duty_cycles[2] * PWM_PERIOD_CYCLES) / 2;
}

void gatedrv_get_phase_currents(gatedriver_t *drv, float phase_currents[3])
{
	// Assumes injected mode conversion is done, convert sample to current in amps and queue data
	// reading from the injected data registers also clears the complete interrupt allowing for it to occur again
	phase_currents[0] = (float) (drv->phase_adc->Instance->JDR1 & ADC_MAX_VALUE);
	phase_currents[1] = (float) (drv->phase_adc->Instance->JDR2 & ADC_MAX_VALUE);
	phase_currents[2] = (float) (drv->phase_adc->Instance->JDR3 & ADC_MAX_VALUE);

	if (!drv->initial_reading_taken) {
		// initial reading defines offset
		drv->channel_offsets[0] = phase_currents[0];
		drv->channel_offsets[1] = phase_currents[1];
		drv->channel_offsets[2] = phase_currents[2];

		drv->initial_reading_taken = 1;
	}

	phase_currents[0] = SAMPLE_TO_AMPS * (phase_currents[0] - drv->channel_offsets[0]);
	phase_currents[1] = SAMPLE_TO_AMPS * (phase_currents[1] - drv->channel_offsets[1]);
	phase_currents[2] = SAMPLE_TO_AMPS * (phase_currents[2] - drv->channel_offsets[2]);
}

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv)
{

}
