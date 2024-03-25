#include "exti.h"

#include <stdint.h>
#include <stddef.h>

#define NUM_EXTI_PINS 16

struct callback_data {
    exti_callback_t cb;
    void *param;
};

static struct callback_data callbacks[NUM_EXTI_PINS] = {0};

// Overload weak EXTI callback function
void HAL_GPIO_EXTI_Callback(uint16_t pin_mask)
{
    // loop through callbacks and call them if the bit is set
    for (int i = 0; i < NUM_EXTI_PINS; i++) {
        if (pin_mask & (1 << i) && callbacks[i].cb != NULL) {
            callbacks[i].cb(callbacks[i].param);
        }
    }
}

void exti_register_callback(uint16_t pin_mask, exti_callback_t callback, void *param)
{
    if (pin_mask == 0)
        return;

    // get index of set bit in pin_mask
    // stolen from https://stackoverflow.com/questions/14767308/how-to-compute-log-base-2-using-bitwise-operators
    int i = 0;
    while (pin_mask >>= 1) i++;

    callbacks[i].cb = callback;
    callbacks[i].param = param;
}