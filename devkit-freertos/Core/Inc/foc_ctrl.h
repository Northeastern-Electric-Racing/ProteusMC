#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t start_time;
	uint32_t end_time;
	bool active;
	bool completed;
} nertimer_t;

typedef enum {
    SECTOR_1 = 0,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6,
} sector_t;

typedef enum {
	FOC_OPEN_LOOP,
	FOC_CLOSED_LOOP,
} foc_mode_t;

/* Struct to contain parameters for field oriented control */
typedef struct {
	float ref_current;
	float dc_bus_voltage;
	float rotor_position;
	float open_loop_ramp_position;
    float rotor_velocity;
	float open_loop_amplitude;
    uint32_t last_run_us;
	nertimer_t closed_loop_timer; // Stay in closed loop for a while
	foc_mode_t mode;

	PID_Obj q_pid;
	PID_Obj d_pid;
	CLARKE_Obj clarke_transform;
	PARK_Obj park_transform;
	IPARK_Obj ipark_transform;
	SVGEN_Obj svm;
} foc_ctrl_t;

void foc_ctrl_init(foc_ctrl_t *controller);

void foc_ctrl_update_v_ref(foc_ctrl_t *controller, float v_ref);

void foc_ctrl_update_encoder(foc_ctrl_t *controller, float velocity, float position);

void foc_ctrl_update_ref_current(foc_ctrl_t *controller, float current);

void foc_ctrl_run(foc_ctrl_t *controller, float phase_currents[3], float duty_cycles[3]);

#endif /* FOC_CTRL_H */
