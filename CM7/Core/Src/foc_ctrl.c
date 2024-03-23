#include "foc_ctrl.h"
#include "proteus_config.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#define INBOUND_QUEUE_SIZE 15

/*
 * CLARKE TRANSFORM PARAMETERS
 * Found from clarke.h
 */
#define NUM_CURRENT_SENSORS     3
#define CLARKE_ALPHA            1/3
#define CLARKE_BETA             1/sqrt(3)

/*
 * SPACE VECTOR MODULATION PARAMETERS
 * Found from svgen.h
 */
#define INV_DC_BUS_VOLTAGE      1 / 600

enum {
    D,
    Q
};

foc_ctrl_t *foc_ctrl_init()
{
    /* Create FOC struct */
    foc_ctrl_t *controller = malloc(sizeof(foc_ctrl_t));
    assert(controller);
    controller->data_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(int[3]), NULL);
    controller->command_queue = osMessageQueueNew(INBOUND_QUEUE_SIZE, sizeof(int[3]), NULL);

    /* Initialize Clarke Transform */
    controller->clarke_transform = malloc(sizeof(CLARKE_Obj));
    assert(controller->clarke_transform);
    CLARKE_setNumSensors(controller->clarke_transform, NUM_CURRENT_SENSORS);
    CLARKE_setScaleFactors(controller->clarke_transform, CLARKE_ALPHA, CLARKE_BETA);

    //TODO: Go into park and ipark and fix sin and cos calls
    /* Initialize Park Transform */
    controller->park_transform = malloc(sizeof(PARK_Obj));
    assert(controller->park_transform);
    PARK_setup(controller->park_transform, 2.0); //TODO: What is "Th"

    /* Initialize Inverse Park Transform */
    controller->ipark_transform = malloc(sizeof(IPARK_Obj));
    assert(controller->ipark_transform);
    IPARK_setup(controller->ipark_transform, 2.0); //TODO: What is "Th"

    /* Initialize Space Vector Modulation */
    controller->svm = malloc(sizeof(SVGEN_Obj));
    assert(controller->svm);
    SVGEN_setup(controller->svm, INV_DC_BUS_VOLTAGE);

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
    if (!controller->command_queue)
        return -1;

    return osMessageQueueGet(controller->command_queue, duty_cycles, NULL, osWaitForever);
}

void vFOCctrl(void *pv_params)
{
    osStatus_t status;

    int16_t phase_buf[3];
    float phase_currents[3];
    float alpha_beta[2];
    float id_iq[2];
    float id_iq_ref[2] = {0, 0}; // Reference d-axis and q-axis currents
    float id_iq_pid[2];
    int16_t calc_cmd[3];

    foc_ctrl_t *controller = (foc_ctrl_t *)pv_params;
    assert(controller);

    for (;;)
    {
        /* Wait until a message is in the queue, send messages when they are in the queue */
        status = osMessageQueueGet(controller->data_queue, phase_buf, NULL, osWaitForever);
        if (status == osOK)
        {
            //TODO: Convert raw ADC reading of phases to current values
            CLARKE_run(controller->clarke_transform, phase_currents, alpha_beta);
            PARK_run(controller->park_transform, alpha_beta, id_iq);

            //TODO: Here, adjust I_q based on the desired current, reference for I_d is always 0
            //TODO: Set new value to id_iq_reg[Q]

            PID_run_parallel(controller->q_pid, id_iq_ref[D], id_iq[D], 0, &id_iq_pid[D]);
            PID_run_parallel(controller->d_pid, id_iq_ref[Q], id_iq[Q], 0, &id_iq_pid[Q]);

            IPARK_run(controller->ipark_transform, id_iq, alpha_beta);
            SVGEN_run(controller->svm, alpha_beta, calc_cmd);

            /* Publish to Onboard Temp Queue */
            osMessageQueuePut(controller->command_queue, calc_cmd, 0U, 0U);
        }

        /* Yield to other tasks */
        osThreadYield();
    }
}