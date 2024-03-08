#ifndef GATEDRIVER_H
#define GATEDRIVER_H

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include <stdbool.h>
#include <stdint.h>

enum phase{
    U,
    V,
    W
};

typedef struct {
	TIM_HandleTypeDef* tim;
    osMutexId_t* tim_mutex;
    TIM_OC_InitTypeDef* pPWMConfig;
	uint32_t pulses[];
} gatedriver_t;

gatedriver_t* gatedrv_init(TIM_HandleTypeDef* tim);

int16_t gatedrv_read_dc_voltage(gatedriver_t* drv);

int16_t gatedrv_read_dc_current(gatedriver_t* drv);

/* Note: This has to atomically write to ALL PWM registers */
//TODO: mechanism for PWM synchronization
int16_t gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[]);

int16_t gatedrv_read_igbt_temp(gatedriver_t* drv);

#endif /* GATEDRIVER_H */
