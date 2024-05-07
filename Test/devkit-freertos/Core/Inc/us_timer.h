#pragma once

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
