#include "us_timer.h"

#include "stm32f3xx.h"
#include "stm32f3xx_hal_tim.h"

static TIM_HandleTypeDef *timer;

void us_timer_init(TIM_HandleTypeDef *htim)
{
    timer = htim;
    HAL_TIM_Base_Start(timer);
}

uint32_t us_timer_get()
{
    // the count is prescaled in CubeMX to increment once per microsecond, so the count corresponds to microseconds
    return timer->Instance->CNT;
}
