#include "ipcc.h"
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

/* Shared memory between cores, manually defined in linker script */
__attribute__((section(".CM4_SHARED_MEM"))) volatile ipcc_msg_t cm4_shared_mem;
__attribute__((section(".CM7_SHARED_MEM"))) volatile ipcc_msg_t cm7_shared_mem;

void ipcc_init(ipcc_t *ipcc, DMA_HandleTypeDef *tx_dma, int tx_hsem_id,
	       int rx_hsem_id)
{
	/* Ensure TX channel works */
	ipcc->tx_dma = tx_dma;

	/* Save Hardware Semaphore IDs */
	ipcc->tx_sem_id = tx_hsem_id;
	ipcc->rx_sem_id = rx_hsem_id;

	assert(HAL_HSEM_FastTake(ipcc->tx_sem_id) == HAL_OK);

	/* Set address of local memory */
	ipcc->rx_mem = &cm7_shared_mem;
	ipcc->tx_mem = &cm4_shared_mem;

	/* Handshake of semaphores */
	while (!HAL_HSEM_IsSemTaken(ipcc->rx_sem_id)) {
	}
	HAL_HSEM_Release(ipcc->tx_sem_id, 1);
}

HAL_StatusTypeDef ipcc_transfer(ipcc_t *ipcc, ipcc_msg_t *msg)
{
	/* Make sure we are clear to send another message */
	//if (HAL_HSEM_FastTake(ipcc->rx_sem_id) == HAL_ERROR)
	//    return HAL_BUSY;

	/* Start DMA transfer to other processor */
	return HAL_DMA_Start_IT(ipcc->tx_dma, (uint32_t)msg,
				(uint32_t)ipcc->tx_mem, sizeof(ipcc_msg_t));
}

void ipcc_signal(ipcc_t *ipcc)
{
	/* Notify other processor */
	HAL_HSEM_Release(ipcc->tx_sem_id, 1);
	HAL_HSEM_ActivateNotification(ipcc->tx_sem_id);
	/* Note that other processor should clear the flag to send another message */
}

void ipcc_receive(ipcc_t *ipcc)
{
	/* Queue to the correct task */
	if (HAL_HSEM_FastTake(ipcc->tx_sem_id) == HAL_ERROR)
		return;

	//printf("Message received:\tID: %d", ipcc->rx_mem->id);

	/* Acknowledge finishing transaction */
	HAL_HSEM_Release(ipcc->tx_sem_id, 1);
}
