#include "foc_ctrl.h"
#include "proteus_config.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "stm32f3xx_hal.h"

#define INBOUND_QUEUE_SIZE 30
#define OUTBOUND_QUEUE_SIZE 15

/*
 * CLARKE TRANSFORM PARAMETERS
 * Found from clarke.h
 */
#define NUM_CURRENT_SENSORS     3
#define CLARKE_ALPHA            1/3
#define CLARKE_BETA             1/sqrt(3)

#define OPEN_LOOP_MAX_AMPLITUDE	0.05	/* max duty cycle */
#define OPEN_LOOP_RAMP_TIME		1		/* seconds */

enum {
	D,
	Q
};

foc_ctrl_t *foc_ctrl_init()
{
	/* Create FOC struct */
	foc_ctrl_t *controller = malloc(sizeof(foc_ctrl_t));
	assert(controller);
	controller->data_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(foc_data_t), NULL);
	controller->command_queue = osMessageQueueNew(OUTBOUND_QUEUE_SIZE, sizeof(float[3]), NULL);

	/* Initialize Clarke Transform */
	controller->clarke_transform = malloc(sizeof(CLARKE_Obj));
	assert(controller->clarke_transform);
	CLARKE_setNumSensors(controller->clarke_transform, NUM_CURRENT_SENSORS);
	CLARKE_setScaleFactors(controller->clarke_transform, CLARKE_ALPHA, CLARKE_BETA);

	/* Initialize Park Transform */
	controller->park_transform = malloc(sizeof(PARK_Obj));
	assert(controller->park_transform);

	/* Initialize Inverse Park Transform */
	controller->ipark_transform = malloc(sizeof(IPARK_Obj));
	assert(controller->ipark_transform);

	/* Initialize Space Vector Modulation */
	controller->svm = malloc(sizeof(SVGEN_Obj));
	assert(controller->svm);

	/* Iniitialize PIDs */
	//TODO: Actually initialize values of PID
	controller->d_pid = malloc(sizeof(PID_Obj));
	assert(controller->d_pid);
	//PID_setGains(controller->d_pid, kp, ki, kd);
	//PID_setMinMax(controller->d_pid);

	controller->q_pid = malloc(sizeof(PID_Obj));
	assert(controller->q_pid);
	//PID_setGains(controller->q_pid, kp, ki, kd);
	//PID_setMinMax(controller->q_pid);

	controller->last_run_ms = HAL_GetTick();

	return controller;
}

osStatus_t foc_queue_frame(foc_ctrl_t *controller, foc_data_t *data)
{
	if (!controller->data_queue)
		return -1;

	return osMessageQueuePut(controller->data_queue, data, 0U, 0U);
}

osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, float duty_cycles[3])
{
	if (!controller->command_queue)
		return -1;

	return osMessageQueueGet(controller->command_queue, duty_cycles, NULL, osWaitForever);
}

static void open_loop_ctrl(foc_ctrl_t *controller, foc_data_t *msg)
{
	/* Note: assuming we start from 0 angular velocity */
	uint16_t duty_cmds[3];

	if (msg->type == FOCDATA_ROTOR_POSITION) {
		/* Assuming tick is set to 1 ms */
		uint32_t dt = HAL_GetTick() - controller->last_run_ms;

		/* Ramp amplitude */
		controller->open_loop_amplitude += ((OPEN_LOOP_MAX_AMPLITUDE - 0) / OPEN_LOOP_RAMP_TIME) * dt;

		/* Clamp signal */
		if (controller->open_loop_amplitude > OPEN_LOOP_MAX_AMPLITUDE)
			controller->open_loop_amplitude = OPEN_LOOP_MAX_AMPLITUDE;

		/* Generate three-phase duty cycles */
		duty_cmds[0] = controller->open_loop_amplitude * sinf(controller->rotor_position);
		duty_cmds[1] = controller->open_loop_amplitude * sinf(controller->rotor_position - 2 * M_PI / 3);
		duty_cmds[2] = controller->open_loop_amplitude * sinf(controller->rotor_position + 2 * M_PI / 3);

		osMessageQueuePut(controller->command_queue, duty_cmds, 0U, 0U);
	}
}

static void closed_loop_ctrl(foc_ctrl_t *controller, foc_data_t *msg)
{
	/* Intermediate values for calculation */
	float calc_cmd[3];
	float phase_currents[3];
	float alpha_beta[2];
	float id_iq[2];
	float id_iq_ref[2] = {0, 0};
	float id_iq_pid[2];
	float v_abc_pu[3];

	return;

	/* Decode message data */
	switch (msg->type) {
		/* If its a phase current measurement, run control pipeline */
		case FOCDATA_PHASE_CURRENT:
			//TODO: Convert raw ADC reading of phases to current values
			CLARKE_run(controller->clarke_transform, msg->payload.phase_currents, alpha_beta);
			PARK_run(controller->park_transform, alpha_beta, id_iq);

			/*
			* Here, adjust I_q based on the desired current, reference for I_d is always 0
			* I_q is actually being varied
			*/
			PID_run_parallel(controller->q_pid, 0, id_iq[D], 0, &id_iq_pid[D]);
			PID_run_parallel(controller->d_pid, controller->ref_current, id_iq[Q], 0, &id_iq_pid[Q]);

			IPARK_run(controller->ipark_transform, id_iq, alpha_beta);
			SVGEN_run(controller->svm, alpha_beta, v_abc_pu);

			calc_cmd[0] = (v_abc_pu[0] + 1.0) / 2.0;
			calc_cmd[1] = (v_abc_pu[1] + 1.0) / 2.0;
			calc_cmd[2] = (v_abc_pu[2] + 1.0) / 2.0;

			/* Send out duty cycle command */
			osMessageQueuePut(controller->command_queue, calc_cmd, 0U, 0U);
			break;

		/* If its a new rotor position, update Park and Inverse Park Transforms */
		case FOCDATA_ROTOR_POSITION:
			PARK_setup(controller->park_transform, msg->payload.rotor_position);
			IPARK_setup(controller->ipark_transform, msg->payload.rotor_position);
			controller->rotor_position = msg->payload.rotor_position;
			break;

		/* If its a new DC bus voltage reading, update the Space Vector Modulation block */
		case FOCDATA_DC_BUS_VOLTAGE:
			SVGEN_setup(controller->svm, 1/msg->payload.dc_bus_voltage);
			controller->dc_bus_voltage = msg->payload.dc_bus_voltage;
			break;

		/* If its a new desired current, update the new reference current */
		case FOCDATA_REF_CURRENT:
			controller->ref_current = msg->payload.ref_current;
			break;

		default:
			/* Unknown data type */
			break;
	}
}

void vFOCctrl(void *pv_params)
{
	osStatus_t status;

	/* Source data for calculations */
	foc_data_t msg;

	foc_ctrl_t *controller = (foc_ctrl_t *)pv_params;
	controller->rotor_position = 0.0;
	controller->last_run_ms = HAL_GetTick();
	assert(controller);

	for (;;)
	{
		/* Wait until a message is in the queue, send messages when they are in the queue */
		status = osMessageQueueGet(controller->data_queue, &msg, NULL, osWaitForever);
		if (status == osOK) {
            if (msg.type == FOCDATA_ROTOR_POSITION) {
                controller->rotor_speed = (controller->rotor_position - msg.payload.rotor_position) / fmax(0.001, ((HAL_GetTick() - controller->last_run_ms) / 1000.0));
            }

            if (abs(controller->rotor_speed) <= 0.1) {
                /* Open Loop Startup Procedure */
                open_loop_ctrl(controller, &msg);
            }
            else {
                /* Closed loop control while running */
                closed_loop_ctrl(controller, &msg);
            }

            controller->last_run_ms = HAL_GetTick();
			controller->rotor_position = msg.payload.rotor_position;
		}

		/* Yield to other tasks */
		osThreadYield();
	}
}