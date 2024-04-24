#ifndef GATEDRIVER_H
#define GATEDRIVER_H

#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define ADV_TIM_CLK_MHz 144
#define SYSCLK_FREQ 72000000
#define TIM_CLOCK_DIVIDER 1
#define ADC_CLK_MHz 72
#define HALL_TIM_CLK 72000000
#define APB1TIM_FREQ 72000000
#define PWM_FREQUENCY 30000
#define PWM_FREQ_SCALING 1
#define PWM_PERIOD_CYCLES (ADV_TIM_CLK_MHz*1000000/PWM_FREQUENCY)&0xFFFE
#define REGULATION_EXECUTION_RATE 1
#define REP_COUNTER REGULATION_EXECUTION_RATE*2-1
#define HTMIN 1
#define PERIOD_VALUE  ((PWM_PERIOD_CYCLES) / 2)

/*
 * Note that these phases readings should ALWAYS be mapped to the corresponding indices
 * Ensure the ADC DMA is mapped the same across boards
 */
enum {
    GATEDRV_PHASE_U,
    GATEDRV_PHASE_V,
    GATEDRV_PHASE_W,
    GATEDRV_NUM_PHASES
};

enum {
    GATEDRV_DC_CURRENT = GATEDRV_NUM_PHASES, /* Keep index rolling from phase enum */
    GATEDRV_IGBT_TEMP,
    GATEDRV_SIZE_OF_ADC_DMA
};

/* Definition of gatedriver struct */
typedef struct {
	TIM_HandleTypeDef* tim;
	ADC_HandleTypeDef* phase_adc;
    TIM_OC_InitTypeDef pwm_cfg;

    int initial_reading_taken;
    float channel_offsets[3];
} gatedriver_t;

/* initialize a new gatedriver */
void gatedrv_init(gatedriver_t *gatedriver, TIM_HandleTypeDef* tim, ADC_HandleTypeDef *phase_adc);

/* Note: This has to atomically write to ALL PWM registers */
void gatedrv_write_pwm(gatedriver_t* drv, float duty_cycles[]);

/* read the internal IGBT temp */
int16_t gatedrv_read_igbt_temp(gatedriver_t* drv);

void gatedrv_get_phase_currents(gatedriver_t *drv, float phase_currents[3]);

#endif /* GATEDRIVER_H */
