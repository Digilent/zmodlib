/**
 * @file dma.h
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief Function declarations used for DMA transfer.
 */

#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>


/**
 * Direction of a DMA transfer.
 */
enum dma_direction {
	DMA_DIRECTION_TX, ///< TX transfer
	DMA_DIRECTION_RX, ///< RX transfer
};

uint32_t fnInitDMA(uintptr_t addr, enum dma_direction direction, int dmaInterrupt);
void fnDestroyDMA(uintptr_t addr);
int fnOneWayDMATransfer(uintptr_t addr, uint32_t *buf, size_t length);
uint8_t fnIsDMATransferComplete(uintptr_t addr);
void* fnAllocBuffer(uintptr_t addr, size_t size);
void fnFreeBuffer(uintptr_t addr, void *buf, size_t size);

#endif /* DMA_H_ */
