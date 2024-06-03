#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include <stdint.h>
#include "cmsis_os.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"
#include "pid.h"

typedef float pwm_signal_t;

typedef struct {
	enum {
		FOCDATA_PHASE_CURRENT,
		FOCDATA_ROTOR_POSITION,
		FOCDATA_DC_BUS_VOLTAGE,
		FOCDATA_REF_CURRENT
	} type;

	union {
		float phase_currents[3];
		float rotor_position;
		float dc_bus_voltage;
		float ref_current;
	} payload;
} foc_data_t;

/* Struct to contain parameters for field oriented control */
typedef struct {
	osThreadId_t thread;
	osMessageQueueId_t data_queue;
	osMessageQueueId_t command_queue;
	float ref_current;
	float dc_bus_voltage;
	float rotor_position;
	float open_loop_amplitude;
	float open_loop_ramp_position;
	float open_loop_ramp_velocity;
	float open_loop_ramp_jork;
	uint32_t last_run_ms;

	PID_Obj *q_pid;
	PID_Obj *d_pid;
	CLARKE_Obj *clarke_transform;
	PARK_Obj *park_transform;
	IPARK_Obj *ipark_transform;
	SVGEN_Obj *svm;
} foc_ctrl_t;

void foc_ctrl_init(foc_ctrl_t *controller);

/* Enqueue a single frame of controller observation */
osStatus_t foc_queue_frame(foc_ctrl_t *controller, foc_data_t *phase_currents);

/* Wait for a command to be sent from the controller */
osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, pwm_signal_t duty_cycles[3]);

extern const osThreadAttr_t foc_ctrl_attributes;

void vFOCctrl(void *pv_params);

#endif /* FOC_CTRL_H */
