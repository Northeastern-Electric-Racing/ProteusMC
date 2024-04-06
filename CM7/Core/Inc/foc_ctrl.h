#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include <stdint.h>
#include "cmsis_os.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"
#include "pid.h"

typedef struct {
	enum {
		FOCDATA_PHASE_CURRENT,
		FOCDATA_ROTOR_POSITION,
		FOCDATA_DC_BUS_VOLTAGE,
		FOCDATA_REF_CURRENT
	} type;

	union {
		int16_t phase_currents[3];
		float rotor_position;
		float dc_bus_voltage;
		float ref_current;
	} payload;
} foc_data_t;

typedef float pwm_signal_t;

/* Struct to contain parameters for field oriented control */
typedef struct {
	osThreadId_t foc_ctrl;
	osMessageQueueId_t data_queue;
	osMessageQueueId_t command_queue;
	float ref_current;
	float dc_bus_voltage;
	float rotor_position;
	uint32_t last_run_ms;
	float open_loop_amplitude;

	//static const float K = /* constant value used in your control system */;
	//static const float ramp_time = 3.0f; // example ramp time in seconds
	//static const float correction_factor = /* your correction factor */;
	//static float speed_ref_rpm; // the reference speed in RPM
	//static float amplitude; // amplitude for sine wave generation

	PID_Obj *q_pid;
	PID_Obj *d_pid;
	CLARKE_Obj *clarke_transform;
	PARK_Obj *park_transform;
	IPARK_Obj *ipark_transform;
	SVGEN_Obj *svm;
} foc_ctrl_t;

foc_ctrl_t *foc_ctrl_init();

/* Enqueue a single frame of controller observation */
osStatus_t foc_queue_frame(foc_ctrl_t *controller, foc_data_t *phase_currents);

/* Wait for a command to be sent from the controller */
osStatus_t foc_retrieve_cmd(foc_ctrl_t *controller, pwm_signal_t duty_cycles[3]);

const osThreadAttr_t foc_ctrl_attributes = {
	.name = "FOC Controller",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityHigh3,
};

void vFOCctrl(void *pv_params);

#endif /* FOC_CTRL_H */
