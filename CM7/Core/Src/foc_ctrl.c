#include "foc_ctrl.h"
#include "proteus_config.h"
#include "state_machine.h"
#include "stm32h7xx_hal.h"
#include "us_timer.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#define INBOUND_QUEUE_SIZE 15
#define OUTBOUND_QUEUE_SIZE 15

/*
 * CLARKE TRANSFORM PARAMETERS
 * Found from clarke.h
 */
#define NUM_CURRENT_SENSORS     3
#define CLARKE_ALPHA            1/3
#define CLARKE_BETA             1/sqrtf(3)

#define OPEN_LOOP_MAX_AMPLITUDE	0.7f	/* max duty cycle */
#define OPEN_LOOP_RAMP_TIME		5.0		/* seconds */
#define OPEN_LOOP_VELOCITY      1000

enum {
	D,
	Q
};

void foc_ctrl_init(foc_ctrl_t *controller)
{
	controller->data_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(foc_data_t), NULL);
	controller->command_queue = osMessageQueueNew(OUTBOUND_QUEUE_SIZE, sizeof(pwm_signal_t[3]), NULL);

	/* Initialize Clarke Transform */
	CLARKE_setNumSensors(&controller->clarke_transform, NUM_CURRENT_SENSORS);
	CLARKE_setScaleFactors(&controller->clarke_transform, CLARKE_ALPHA, CLARKE_BETA);

	/* Initialize PIDs */
	PID_setGains(&controller->d_pid, 0.1, 0.1, 0);
	PID_setMinMax(&controller->d_pid, 0.0, 1.0);
	PID_init(&controller->d_pid, sizeof(controller->d_pid));

	PID_setGains(&controller->q_pid, 0.1, 0.1, 0);
	PID_setMinMax(&controller->q_pid, 0.0, 2.0);
	PID_init(&controller->q_pid, sizeof(controller->q_pid));

	controller->last_run_us = us_timer_get();
	controller->open_loop_ramp_velocity = 0.3;
}

osStatus_t foc_queue_frame(foc_ctrl_t *controller, foc_data_t *data)
{
	if (!controller->data_queue)
		return -1;

	return osMessageQueuePut(controller->data_queue, data, 0U, 0U);
}

osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, pwm_signal_t duty_cycles[3])
{
	if (!controller->command_queue)
		return -1;

	return osMessageQueueGet(controller->command_queue, duty_cycles, NULL, osWaitForever);
}

static void open_loop_ctrl(foc_ctrl_t *controller)
{
	pwm_signal_t duty_cycles[3];

	/* Note: assuming we start from 0 angular velocity */
	uint32_t current_time_us = us_timer_get();
	uint32_t dt = (uint32_t) ((uint32_t)current_time_us - (uint32_t) controller->last_run_us);
	if ((int32_t)dt < 0) dt = 180; /* Account for wrap around of timer */

	//printf("Time us: %ld\r\n", dt);

	/* Ramp amplitude */
	controller->open_loop_amplitude += ((OPEN_LOOP_MAX_AMPLITUDE ) / OPEN_LOOP_RAMP_TIME) * (dt / 1000000.0);

	/* Clamp signal */
	if (controller->open_loop_amplitude > OPEN_LOOP_MAX_AMPLITUDE)
		controller->open_loop_amplitude = OPEN_LOOP_MAX_AMPLITUDE;

	/* Generate three-phase duty cycles */
	duty_cycles[0] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position) + 1) / 2.0 * 10.0;
	duty_cycles[1] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position - 2.0 * M_PI / 3.0) + 1.0) / 2 * 10.0;
	duty_cycles[2] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position + 2.0 * M_PI / 3.0) + 1.0) / 2 * 10.0;

	controller->last_run_us = current_time_us;
	if (controller->open_loop_ramp_velocity < OPEN_LOOP_VELOCITY)
		controller->open_loop_ramp_velocity *= 1.003;

	controller->open_loop_ramp_position += controller->open_loop_ramp_velocity * (dt / 1000000.0f);
	//printf("U: %ld A, V: %ld A, W: %ld A\r\n",
    //       (uint32_t)(duty_cycles[0] * 1000),
    //       (uint32_t)(duty_cycles[1] * 1000),
    //       (uint32_t)(duty_cycles[2] * 1000));
	osMessageQueuePut(controller->command_queue, duty_cycles, 0U, 0U);
}

static void closed_loop_ctrl(foc_ctrl_t *controller, foc_data_t *msg)
{
	/* Intermediate values for calculation */
	pwm_signal_t calc_cmd[3];
	float phase_currents[3];
	float alpha_beta[2];
	float id_iq[2];
	float id_iq_ref[2] = {0, 0};
	float id_iq_pid[2];
	float v_abc_pu[3];

	/* Decode message data */
	switch (msg->type) {
		/* If its a phase current measurement, run control pipeline */
		case FOCDATA_PHASE_CURRENT:
			//TODO: Convert raw ADC reading of phases to current values
			CLARKE_run(&controller->clarke_transform, msg->payload.phase_currents, alpha_beta);
			PARK_run(&controller->park_transform, alpha_beta, id_iq);

			/*
			* Here, adjust I_q based on the desired current, reference for I_d is always 0
			* I_q is actually being varied
			*/
			PID_run_parallel(&controller->q_pid, 0, id_iq[D], 0, &id_iq_pid[D]);
			PID_run_parallel(&controller->d_pid, controller->ref_current, id_iq[Q], 0, &id_iq_pid[Q]);

			IPARK_run(&controller->ipark_transform, id_iq, alpha_beta);
			SVGEN_run(&controller->svm, alpha_beta, v_abc_pu);

			calc_cmd[0] = (v_abc_pu[0] + 1.0) / 2.0;
			calc_cmd[1] = (v_abc_pu[1] + 1.0) / 2.0;
			calc_cmd[2] = (v_abc_pu[2] + 1.0) / 2.0;

			/* Send out duty cycle command */
			osMessageQueuePut(controller->command_queue, calc_cmd, 0U, 0U);
			break;

		/* If its a new rotor position, update Park and Inverse Park Transforms */
		case FOCDATA_ROTOR_POSITION:
			PARK_setup(&controller->park_transform, msg->payload.rotor_position);
			IPARK_setup(&controller->ipark_transform, msg->payload.rotor_position);
			controller->rotor_position = msg->payload.rotor_position;
			break;

		/* If its a new DC bus voltage reading, update the Space Vector Modulation block */
		case FOCDATA_DC_BUS_VOLTAGE:
			SVGEN_setup(&controller->svm, 1/msg->payload.dc_bus_voltage);
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

const osThreadAttr_t foc_ctrl_attributes = {
	.name = "FOC Controller",
	.stack_size = 128 * 8,
	/* Note this task's priority is low since it happens so frequently, allows other, less frequent tasks to go */
	.priority = (osPriority_t)osPriorityLow,
};

void vFOCctrl(void *pv_params)
{
	osStatus_t status;

	/* Source data for calculations */
	foc_data_t msg;
	pwm_signal_t duty_cycles[3];

	foc_ctrl_t *controller = (foc_ctrl_t *)pv_params;
	assert(controller);

	for (;;)
	{
		/* Wait until a message is in the queue, send messages when they are in the queue */
		status = osMessageQueueGet(controller->data_queue, &msg, NULL, osWaitForever);
		if (status == osOK) {
			switch (get_state()) {
				case START:
					/* Open Loop Startup Procedure */
					open_loop_ctrl(controller);
					break;
				case START_RUN:
					/* Ensure ready to begin closed loop control */
					//TODO: this
					break;
				case RUN:
					/* Closed loop control while running */
					closed_loop_ctrl(controller, &msg);
					break;
				case LV_BOOT:
				case CHARGE_BOOT_CAP:
				case OFFSET_CALIB:
				case CLEAR:
				case STOP_IDLE:
				case FAULTED:
					/* Do Nothing, Ensure signals are safe */
					controller->open_loop_amplitude = 0.0;
					controller->last_run_us = us_timer_get();
					//TODO: this
					break;
				case STOP_NOW:
					/* Stop Motor Safely */
					//TODO: this
					break;
				default:
					/* Unknown State */
					break;
			}
			//uint32_t first_time = us_timer_get();
			//closed_loop_ctrl(controller, &msg);
			open_loop_ctrl(controller);
			//printf("Time us:%ld\r\n", us_timer_get() - first_time);
		}

		/* Yield to other tasks */
		osThreadYield();
	}
}