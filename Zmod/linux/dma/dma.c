/**
 * @file linux/dma/dma.c
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of platform-specific methods for DMA devices.
 */

#ifdef LINUX_APP

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <dirent.h>
#include <string.h>

#include "../../dma.h"
#include "../utils.h"
#include "libaxidma.h"

/**
 * Struct containing data specific to this DMA instance.
 */
typedef struct _dma_env {
	uintptr_t addr; ///< the physical address of the DMA device
	enum dma_direction direction; ///< the direction of the DMA transfer
	axidma_dev_t dma_inst; ///< a pointer to the instance of the AXI DMA library
	int channel_id; ///< the channel id that will be used for DMA transfers
	uint8_t complete_flag; ///< whether the current DMA transfer is complete or not
} DMAEnv;

/**
 * Call when a DMA interrupt occurs.
 *
 * @param channel_id the id of the channel for which the DMA interrupt occurs
 * @param data a pointer to the DMAEnv instance used
 */
void fnDMAInterruptHandler(int channel_id, void *data)
{
	DMAEnv *dma_env = (DMAEnv *)data;
	if (!dma_env)
		return;

	dma_env->complete_flag = 1;
}

/**
 * Start a one-way DMA transfer.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 * @param buf the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate the requested transfer
 * @param transfer_size the size of the transfer in bytes
 *
 * @return 0 on success, any other number on failure
 */
int fnOneWayDMATransfer(uintptr_t addr, uint32_t *buf, size_t transfer_size)
{
	DMAEnv *dma_env = (DMAEnv *)addr;
	if (!dma_env)
		return -1;

	dma_env->complete_flag = 0;

	return axidma_oneway_transfer(dma_env->dma_inst, dma_env->channel_id,
			(void *)buf, transfer_size, 0);
}

/**
 * Check if the DMA transfer previously started has completed.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 *
 * @return 1 if the DMA transfer completed, 0 if it is still running
 */
uint8_t fnIsDMATransferComplete(uintptr_t addr)
{
	DMAEnv *dma_env = (DMAEnv *)addr;
	if (!dma_env)
		return 0;

	return dma_env->complete_flag;
}

#define NODES_DIRECTORY "/sys/firmware/devicetree/base/amba_pl"

/**
 * Find the chardev index given the DMA address.
 *
 * @param dma_addr the DMA address to look for
 *
 * @return the chardev index if found, -1 if not
 */
int fnGetCharDevIndexForDmaPhandle(uint32_t dma_addr) {
	DIR *directory = opendir(NODES_DIRECTORY);
	char path[MAX_PATH_SIZE];

	if (!directory) {
		return -1;
	}

	uint32_t dma_phandle = -1;
	struct dirent *directory_entry;
	while ((directory_entry = readdir(directory)) != NULL) {
		// Check if the current directory is a DMA one
		bool is_dma = fnStartsWith(directory_entry->d_name, "dma");
		if (!is_dma) {
			continue;
		}

		// Construct full path of the DMA directory
		snprintf(path, MAX_PATH_SIZE, "%s/%s",
				NODES_DIRECTORY, directory_entry->d_name);

		// Check if the DMA address is the one we're looking for
		uint32_t addr = fnReadUint32(path, "reg");
		if (dma_addr != addr) {
			continue;
		}

		// Read phandle value
		dma_phandle = fnReadUint32(path, "phandle");
		break;
	}

	rewinddir(directory);

	uint32_t chardev_index = -1;
	uint32_t phandle;
	while ((directory_entry = readdir(directory)) != NULL) {
		// Check if the current directory is a DMA character device one
		bool is_chardev = fnStartsWith(directory_entry->d_name, "axidma_chrdev");
		if (!is_chardev) {
			continue;
		}

		// Construct full path of the DMA character device directory
		snprintf(path, MAX_PATH_SIZE, "%s/%s",
				NODES_DIRECTORY, directory_entry->d_name);

		// Check if the DMA phandle is the one we're looking for
		phandle = fnReadUint32(path, "dmas");
		if (dma_phandle != phandle) {
			continue;
		}

		// Read phandle DMA character device index
		chardev_index = fnReadUint32(path, "index");
		break;
	}

	// Close directory stream
	closedir(directory);
	return chardev_index;
}

/**
 * Initialize a DMAEnv instance.
 *
 * @param addr the physical address of the DMA device
 * @param direction the direction of the DMA transfer
 * @param dmaInterrupt the number of the DMA device interrupt,
 *  unused on Linux platforms
 *
 * @return the address of the DMAEnv instance
 */
uint32_t fnInitDMA(uintptr_t addr, enum dma_direction direction, int dmaInterrupt)
{
	// Find DMA character device index
	int index = fnGetCharDevIndexForDmaPhandle(addr);
	if (index < 0) {
		return 0;
	}

	// Allocate DMA environment structure
	DMAEnv *dma_env = (DMAEnv *)malloc(sizeof(DMAEnv));
	if (dma_env < 0) {
		return 0;
	}

	// Initialize DMA
	dma_env->dma_inst = axidma_init_dev(index);
	if (!dma_env->dma_inst) {
        return 0;
	}

	dma_env->addr = addr;
	dma_env->direction = direction;

    // Get channels
	const array_t *channels;
	if (direction == DMA_DIRECTION_TX) {
		channels = axidma_get_dma_tx(dma_env->dma_inst);
	} else {
		channels = axidma_get_dma_rx(dma_env->dma_inst);
	}

	if (channels->len < 1) {
		return 0;
	}

    dma_env->channel_id = channels->data[0];

    // Set callback handler
    axidma_set_callback(dma_env->dma_inst, dma_env->channel_id, fnDMAInterruptHandler, dma_env);

    return (uint32_t)dma_env;
}

/**
 * Allocate a DMA buffer.
 *
 * @param addr the physical address of the DMA device
 * @param size the size of the DMA buffer
 *
 * @return the address of the newly allocated DMA buffer
 */
void* fnAllocBuffer(uintptr_t addr, size_t size) {
	DMAEnv *dma_env = (DMAEnv *)addr;
	if (!dma_env)
		return NULL;

	uint32_t *buf = (uint32_t *)axidma_malloc(dma_env->dma_inst, size);
	if (!buf) {
		return NULL;
	}

	return buf;
}

/**
 * Free a DMA buffer.
 *
 * @param addr the physical address of the DMA device
 * @param buf the address of the DMA buffer
 * @param size the size of the DMA buffer
 */
void fnFreeBuffer(uintptr_t addr, void *buf, size_t size) {
	DMAEnv *dma_env = (DMAEnv *)addr;
	if (!dma_env)
		return;

	axidma_free(dma_env->dma_inst, buf, size);
}

/**
 * Destroy a DMAEnv instance.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 */
void fnDestroyDMA(uintptr_t addr)
{
	DMAEnv *dma_env = (DMAEnv *)addr;
	if (!dma_env)
		return;

	axidma_stop_transfer(dma_env->dma_inst, dma_env->channel_id);

	axidma_destroy(dma_env->dma_inst);
}

#endif // LINUX_APP
