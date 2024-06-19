#ifndef IPCC_H
#define IPCC_H

#include "stm32h7xx_hal.h"
#include "proteus_config.h"
#include "fdcan.h"

typedef struct {
	enum {
		IPCC_CAN_MSG,
		IPCC_UART_MSG,
		IPCC_STATE_TRANSITION,
		IPCC_FAULT_ALERT
	} id;
	union {
		uint32_t can;
		char serial[PRINTF_BUFFER_LEN];
		state_t state;
		fault_data_t fault;
	} msg;
} ipcc_msg_t;

typedef struct {
	DMA_HandleTypeDef *tx_dma;
	volatile ipcc_msg_t *rx_mem;
	volatile ipcc_msg_t *tx_mem;
	int tx_sem_id; /* For notification of other core */
	int rx_sem_id; /* For acknowledgement of transmission */
} ipcc_t;

/**
 * Initializes the Inter Processor Communication Channel
*/
void ipcc_init(ipcc_t *ipcc, DMA_HandleTypeDef *tx_dma, int tx_hsem_id,
	       int rx_hsem_id);

/**
 * Initiates a transfer of data to the other processor
*/
HAL_StatusTypeDef ipcc_transfer(ipcc_t *ipcc, ipcc_msg_t *msg);

/**
 * Signal to the other core that a message has been passed
*/
void ipcc_signal(ipcc_t *ipcc);

/**
 * Reads what the other processor has sent over and acknowledges transmission
*/
void ipcc_receive(ipcc_t *ipcc);

#endif /* IPC_H */