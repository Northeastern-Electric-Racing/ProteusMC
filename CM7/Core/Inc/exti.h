#ifndef EXTI_H
#define EXTI_H

#include <stdint.h>

typedef void (*exti_callback_t)(void *);

/**
 * @brief Registers callback to run when external interrupt occurs on specified pin
 * 
 */
void exti_register_callback(uint16_t pin_mask, exti_callback_t callback, void *param);

#endif /* EXTI_H */