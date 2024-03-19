#ifndef QUEUES_H
#define QUEUES_H

#include "cmsis_os.h"
#include "gatedriver.h"

#define PHASE_CURRENT_QUEUE_SIZE    8

typedef struct {
    uint32_t current[GATEDRV_NUM_PHASES];
} phase_current_t;

extern osMessageQueueId_t phase_current_queue;

#endif /* QUEUES_H */