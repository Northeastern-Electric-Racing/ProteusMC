#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include <stdint.h>
#include "cmsis_os.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"

/* Struct to contain parameters for field oriented control */
typedef struct {
    osThreadId_t foc_ctrl;
    osMessageQueueId_t data_queue;
    osMessageQueueId_t command_queue;

    CLARKE_Obj *clarke_transform;
    PARK_Obj *park_transform;
    IPARK_Obj *ipark_transform;
    SVGEN_Obj *svm;
} foc_ctrl_t;

foc_ctrl_t *foc_ctrl_init();

/* Enqueue a single frame of controller observation */
osStatus_t foc_queue_frame(foc_ctrl_t *controller, int16_t phase_currents[3]);

/* Wait for a command to be sent from the controller */
osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, int16_t duty_cycles[3]);

const osThreadAttr_t foc_ctrl_attributes = {
    .name = "FOC Controller",
    .stack_size = 128 * 8,
    .priority = (osPriority_t)osPriorityHigh3,
};

void vFOCctrl(void *pv_params);

#endif /* FOC_CTRL_H */
