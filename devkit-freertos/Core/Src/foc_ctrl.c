#include "foc_ctrl.h"
#include "proteus_config.h"
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include "us_timer.h"
#include <stdbool.h>
#include <stdlib.h>

#define CLOSED_LOOP_MIN_DURATION	1000 // ms
#define CLOSED_LOOP_SPEED_THRESHOLD	100.0 // rad/s

/*
 * CLARKE TRANSFORM PARAMETERS
 * Found from clarke.h
 */
#define NUM_CURRENT_SENSORS     3
#define CLARKE_ALPHA            1/3
#define CLARKE_BETA             1/sqrtf(3)

#define OPEN_LOOP_MAX_AMPLITUDE	0.3f	/* max duty cycle */
#define OPEN_LOOP_RAMP_TIME		500000.0		/* microseconds */
#define OPEN_LOOP_VELOCITY      10

void start_timer(nertimer_t *timer, uint32_t duration)
{
    /* this function assumes tick set to default 1 ms. Update or use HAL_GetTickFreq() if not the case */
    timer->start_time = HAL_GetTick();
    timer->end_time = timer->start_time + duration;
    timer->active = true;
    timer->completed = false;
}

void cancel_timer(nertimer_t *timer)
{
    timer->active = false;
    timer->completed = false;
}

bool is_timer_expired(nertimer_t *timer)
{
    if (timer->active)
    {
        if (HAL_GetTick() >= timer->end_time)
        {
            timer->active = false;
            timer->completed = true;
        }
    }
    return timer->completed;
}

bool is_timer_active(nertimer_t *timer)
{
    return timer->active;
}

enum {
	D,
	Q
};

void foc_ctrl_init(foc_ctrl_t *controller)
{
	/* Initialize Clarke Transform */
	CLARKE_setNumSensors(&controller->clarke_transform, NUM_CURRENT_SENSORS);
	CLARKE_setScaleFactors(&controller->clarke_transform, CLARKE_ALPHA, CLARKE_BETA);

	/* Iniitialize PIDs */
	//TODO: Actually initialize values of PID
	PID_setGains(&controller->d_pid, 1, 1, 0);
	//PID_setMinMax(controller->d_pid);
	PID_init(&controller->d_pid, sizeof(controller->d_pid));

	PID_setGains(&controller->q_pid, 1, 1, 0);
	// PID_setMinMax(controller->q_pid);
	PID_init(&controller->q_pid, sizeof(controller->q_pid));

	controller->last_run_us = us_timer_get();
	cancel_timer(&controller->closed_loop_timer);
}

static void open_loop_ctrl(foc_ctrl_t *controller, float duty_cycles[3])
{
	/* Note: assuming we start from 0 angular velocity */
	/* Assuming tick is set to 1 ms */
	uint32_t current_time_us = us_timer_get();
	uint32_t dt = (uint16_t) ((uint16_t) current_time_us - (uint16_t) controller->last_run_us);

	/* Ramp amplitude */
	controller->open_loop_amplitude += ((OPEN_LOOP_MAX_AMPLITUDE ) / OPEN_LOOP_RAMP_TIME) * dt;

	/* Clamp signal */
	if (controller->open_loop_amplitude > OPEN_LOOP_MAX_AMPLITUDE)
		controller->open_loop_amplitude = OPEN_LOOP_MAX_AMPLITUDE;

	/* Generate three-phase duty cycles */
	duty_cycles[0] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position) + 1) / 2;
	duty_cycles[1] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position - 2 * M_PI / 3) + 1) / 2;
	duty_cycles[2] = controller->open_loop_amplitude * (sinf(controller->open_loop_ramp_position + 2 * M_PI / 3) + 1) / 2;

	controller->last_run_us = current_time_us;
	controller->open_loop_ramp_position +=  dt;
}

static void closed_loop_ctrl(foc_ctrl_t *controller, float phase_currents[3], float duty_cycles[3])
{
	/* Intermediate values for calculation */
	float alpha_beta[2];
	float id_iq[2];
	float id_iq_pid[2];
	float v_abc_pu[3];

	CLARKE_run(&controller->clarke_transform, phase_currents, alpha_beta);
	PARK_run(&controller->park_transform, alpha_beta, id_iq);

	/*
	* Here, adjust I_q based on the desired current, reference for I_d is always 0
	* I_q is actually being varied
	*/
	PID_run_parallel(&controller->q_pid, 0, id_iq[D], 0, &id_iq_pid[D]);
	PID_run_parallel(&controller->d_pid, controller->ref_current, id_iq[Q], 0, &id_iq_pid[Q]);

	IPARK_run(&controller->ipark_transform, id_iq, alpha_beta);
	SVGEN_run(&controller->svm, alpha_beta, v_abc_pu);

	duty_cycles[0] = (v_abc_pu[0] + 1.0) / 2.0;
	duty_cycles[1] = (v_abc_pu[1] + 1.0) / 2.0;
	duty_cycles[2] = (v_abc_pu[2] + 1.0) / 2.0;
}

void foc_ctrl_update_v_ref(foc_ctrl_t *controller, float v_ref)
{
	SVGEN_setup(&controller->svm, 1 / v_ref);
	controller->dc_bus_voltage = v_ref;
}

void foc_ctrl_update_encoder(foc_ctrl_t *controller, float velocity, float position)
{
	PARK_setup(&controller->park_transform, position);
	IPARK_setup(&controller->ipark_transform, position);
	controller->rotor_position = position;
	controller->rotor_velocity = velocity;
}

void foc_ctrl_update_ref_current(foc_ctrl_t *controller, float current)
{
	controller->ref_current = current;
}

void foc_ctrl_run(foc_ctrl_t *controller, float phase_currents[3], float duty_cycles[3])
{
	if (abs(controller->rotor_velocity) > CLOSED_LOOP_SPEED_THRESHOLD) {
		start_timer(&controller->closed_loop_timer, CLOSED_LOOP_MIN_DURATION);
	}

	if (abs(controller->rotor_velocity) <= CLOSED_LOOP_SPEED_THRESHOLD && (!is_timer_active(&controller->closed_loop_timer) || is_timer_expired(&controller->closed_loop_timer))) {
		/* Open Loop Startup Procedure */
		controller->mode = FOC_OPEN_LOOP;
		open_loop_ctrl(controller, duty_cycles);
	} else {
		/* Closed loop control while running */
		controller->mode = FOC_CLOSED_LOOP;
		closed_loop_ctrl(controller, phase_currents, duty_cycles);
	}
}
