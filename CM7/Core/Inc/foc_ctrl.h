#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include <stdint.h>
#include "cmsis_os.h"
#include "gatedriver.h"

/* Struct to contain parameters for field oriented control */
typedef struct {
    osThreadId_t foc_ctrl;
    osMessageQueueId_t data_queue;

    /* Note that phase actor task should hang on this for new command */
    osMessageQueueId_t command_queue;
} foc_ctrl_t;

foc_ctrl_t *init_foc_ctrl();

/* Enqueue a single frame of controller observation */
int8_t queue_frame(foc_ctrl_t *controller, int16_t phase_currents[3]);

const osThreadAttr_t foc_ctrl_attributes = {
	.name		= "FOC Controller",
	.stack_size = 128 * 8,
	.priority	= (osPriority_t)osPriorityHigh3,
};

void vFOCctrl(void* pv_params);

#endif /* FOC_CTRL_H */