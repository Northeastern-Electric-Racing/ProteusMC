#include "foc_ctrl.h"

#define INBOUND_QUEUE_SIZE  15

foc_ctrl_t *foc_ctrl_init()
{
	/* Create FOC struct */
	foc_ctrl_t* controller = malloc(sizeof(foc_ctrl_t));
	assert(controller);
	controller->data_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(int[3]), NULL);
	controller->command_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(int[3]), NULL);

	return controller;
}

osStatus_t foc_queue_frame(foc_ctrl_t *controller, int16_t phase_currents[3])
{
	if (!controller->data_queue)
		return -1;

	return osMessageQueuePut(controller->data_queue, phase_currents, 0U, 0U);
}

osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, int16_t duty_cycles[3])
{
	return osMessageQueueGet(controller->command_queue, duty_cycles, NULL, osWaitForever);
}

void vFOCctrl(void* pv_params)
{
	osStatus_t status;

	int16_t phase_buf[3];
	int16_t calc_cmd[3];

	foc_ctrl_t *controller = (foc_ctrl_t *)pv_params;
	assert(controller);

	for (;;) {
		/* Wait until a message is in the queue, send messages when they are in the queue */
		status = osMessageQueueGet(controller->data_queue, phase_buf, NULL, osWaitForever);
		if (status == osOK) {
			//TODO: Execute control pipeline

			/* Publish to Onboard Temp Queue */
			osMessageQueuePut(controller->command_queue, calc_cmd, 0U, 0U);
		}

		/* Yield to other tasks */
		osThreadYield();
	}
}
