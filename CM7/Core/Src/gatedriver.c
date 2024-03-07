
#include "gatedriver.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define  PERIOD_VALUE		(uint32_t)(2000 - 1)

//TODO: Look up STM callback func pointer for ADCs
static void gatedrv_current_adc_cb(gatedriver_t* drv)
{

}

static void gatedrv_voltage_adc_cb(gatedriver_t* drv)
{

}

static void gatedrv_ready_cb(gatedriver_t* drv)
{

}

static void gatedrv_reset_cb(gatedriver_t* drv)
{

}

static void gatedrv_fault_cb(gatedriver_t* drv)
{

}

gatedriver_t* gatedrv_init(TIM_HandleTypeDef* tim)
{
	/* Assert hardware params */
	assert(tim);

	/* Create MPU struct */
	gatedriver_t* gatedriver = malloc(sizeof(gatedriver_t));
	assert(gatedriver);

	/* Set interfaces */
	gatedriver->tim			= tim;
	
	/* Init hardware */
	tim->Init.Prescaler			= 0;
	tim->Init.Period			= PERIOD_VALUE;
	tim->Init.ClockDivision		= 0;
	tim->Init.CounterMode		= TIM_COUNTERMODE_UP;
	tim->Init.RepetitionCounter	= 0;

	/* Initialize the Onboard Temperature Sensor */
	//mpu->temp_sensor = malloc(sizeof(sht30_t));
	//assert(mpu->temp_sensor);
	//mpu->temp_sensor->i2c_handle = hi2c;
	//assert(!sht30_init(mpu->temp_sensor)); /* This is always connected */

	//TODO: Init Mutexes
	/* Create Mutexes */
	//mpu->i2c_mutex = osMutexNew(&mpu_i2c_mutex_attr);
	//assert(mpu->i2c_mutex);

	//TODO: Link interrupts to callbacks

	return gatedriver;
}

int16_t gatedrv_read_dc_voltage(gatedriver_t* drv)
{

}

int16_t gatedrv_read_dc_current(gatedriver_t* drv)
{

}

/* Note: This has to atomically write to ALL PWM registers */
int16_t gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[])
{
	/* Computing pulses */
	uint32_t pulses[3];
	pulses[0] = (uint32_t) (duty_cycles[0] * PERIOD_VALUE / 100);
	pulses[1] = (uint32_t) (duty_cycles[1] * PERIOD_VALUE / 100);
	pulses[2] = (uint32_t) (duty_cycles[2] * PERIOD_VALUE / 100);

	/* Common configuration for all channels */
	TIM_OC_InitTypeDef PWMConfig;
	PWMConfig.OCMode       = TIM_OCMODE_PWM1;
	PWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCIdleState  = TIM_OCIDLESTATE_SET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	PWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;

	/* Attempting to set channel 1 */
	PWMConfig.Pulse = pulses[0];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Do nothing and return */
		return 1;
	}

	/* Attempting to set channel 2 */
	PWMConfig.Pulse = pulses[1];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_2) != HAL_OK)
	{
		/* Attempt to revert last channel change and return */
		PWMConfig.Pulse = drv->pulses[0];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_1);

		return 1;
	}

	/* Attempting to set channel 3 */
	PWMConfig.Pulse = pulses[2];
	if(HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_3) != HAL_OK)
	{
		/* attempt to revert previous channel changes and return */
		PWMConfig.Pulse = drv->pulses[0];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_1);

		PWMConfig.Pulse = drv->pulses[1];
		HAL_TIM_PWM_ConfigChannel(drv->tim, &PWMConfig, TIM_CHANNEL_2);

		return 1;
	}

	/* successful PWM modifications - save configuration and return */
	drv->pulses[0] = pulses[0];
	drv->pulses[1] = pulses[1];
	drv->pulses[2] = pulses[2];

	return 0;
}

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv)
{

}
