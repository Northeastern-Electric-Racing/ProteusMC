#ifndef US_TIMER_H
#define US_TIMER_H

#include <stdint.h>

/**
 * @brief Init microsecond timer
 *
 */
void us_timer_init();

/**
 * @brief Get time in microseconds
 *
 * @return uint32_t
 */
uint32_t us_timer_get();

#endif /* US_TIMER_H */
