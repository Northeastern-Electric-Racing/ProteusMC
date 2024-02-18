
#include "gatedriver.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

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

gatedriver_t* init_gatedriver()
{
    //TODO: Assert hardware params
	//assert(hi2c);
	//assert(accel_adc1);
	//assert(accel_adc2);
	//assert(brake_adc);
	//assert(led_gpio);
	//assert(watchdog_gpio);

	/* Create MPU struct */
	gatedriver_t* gatedriver = malloc(sizeof(gatedriver_t));
	assert(gatedriver);

    //TODO: Set interfaces
	//mpu->hi2c		   = hi2c;
	//mpu->accel_adc1	   = accel_adc1;
	//mpu->accel_adc2	   = accel_adc2;
	//mpu->brake_adc	   = brake_adc;
	//mpu->led_gpio	   = led_gpio;
	//mpu->watchdog_gpio = watchdog_gpio;

    //TODO: Init hardware
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
//TODO: mechanism for PWM synchronization
int16_t gatedrv_write_pwm(gatedriver_t* drv)
{

}

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv)
{

}
