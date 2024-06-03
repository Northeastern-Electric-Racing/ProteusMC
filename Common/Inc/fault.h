#ifndef PROTEUS_FAULT_H
#define PROTEUS_FAULT_H

#include "cmsis_os.h"
#include "proteus_config.h"

/* Function to queue a fault */
int queue_fault(fault_data_t *fault_data);

/* Defining Fault Hanlder Task */
void vFaultHandler(void *pv_params);
extern osThreadId_t fault_handle;
extern const osThreadAttr_t fault_handle_attributes;

#endif // FAULT_H
