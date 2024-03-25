#include "inc_encoder.h"

#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "stm32h7xx.h"

#include "exti.h"

#define ENCODER_MAX_COUNT (1 << 13)

static void inc_encoder_z_pulse_cb(void *param)
{
    // cast void param back into incremental encoder and reset count
    ((inc_encoder_t *) param)->tim->Instance->CNT = 0;
}

inc_encoder_t *inc_encoder_init(TIM_HandleTypeDef *timer_handle, uint16_t z_pin_num) {
    // make sure the handle exists
    assert(timer_handle);
    assert(z_pin_num);

    // allocate new struct
    inc_encoder_t *encoder = malloc(sizeof(inc_encoder_t));
    assert(encoder);

    encoder->tim = timer_handle;
    encoder->z_pin_num = z_pin_num;

    // register callback with ISR for when z pin goes high to reset counter
    exti_register_callback(z_pin_num, &inc_encoder_z_pulse_cb, (void *) encoder);

    return encoder;
}

int16_t inc_encoder_get_angle(inc_encoder_t *encoder, float *angle)
{
    uint32_t counts = encoder->tim->Instance->CNT;

    // convert counts to radians
    *angle = (((float) counts) / ENCODER_MAX_COUNT) * 2 * M_PI;

    return 0;
}