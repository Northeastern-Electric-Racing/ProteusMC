#ifndef PROTEUS_CONFIG_H
#define PROTEUS_CONFIG_H

#define PRINTF_BUFFER_LEN 128 /* Characters */

// TODO: add actual list of states
typedef enum {
    LV_BOOT,
    CHARGE_BOOT_CAP,
    OFFSET_CALIB,
    CLEAR,
    START,      /* Open Loop Startup */
    START_RUN,
    RUN,        /* Closed Loop Control */
    STOP_IDLE,
    STOP_NOW,
    FAULTED,
    MAX_FUNC_STATES
} state_t;

/* Faulting config */
typedef enum
{
    DEFCON1 = 1,
    DEFCON2,
    DEFCON3,
    DEFCON4,
    DEFCON5
} fault_sev_t;

// TODO: add actual fault codes
typedef enum
{
    FAULTS_CLEAR = 0x0,
    MAX_FAULTS
} fault_code_t;

typedef struct
{
    fault_code_t id;
    fault_sev_t severity;
    char diag[PRINTF_BUFFER_LEN];
} fault_data_t;

#endif /* PROTEUS_CONFIG_H */
