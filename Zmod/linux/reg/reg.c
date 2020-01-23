/**
 * @file linux/reg/reg.c
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of the register writing functions.
 */

#ifdef LINUX_APP

#ifdef __cplusplus
extern "C" {
#endif
#include <libuio.h>
#ifdef __cplusplus
}
#endif

#include <dirent.h>
#include <sys/types.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>

#include "../../reg.h"

#define MAX_UIO_DEVICES 16
#define MAX_UIO_MAPS 16
#define MAX_MAP_ADDR_PATH_LEN 255

/**
 * Get an UIO device's number and map number given an address.
 *
 * @param addr the address to look for
 * @param uioNum the pointer at which to store the found UIO device number,
 *  will be set to -1 if none is found
 * @param mapNum the pointer at which to store the found UIO map number,
 *  will be set to -1 if none is found
 */
void getDeviceByAddr(uintptr_t addr, int *uioNum, int *mapNum) {
	char mapAddrPath[MAX_MAP_ADDR_PATH_LEN];
	uintptr_t mapAddr = 0;
	int uio, map;

	for (uio = 0; uio < MAX_UIO_DEVICES; uio++) {
		for (map = 0; map < MAX_UIO_MAPS; map++) {
			snprintf(mapAddrPath, MAX_MAP_ADDR_PATH_LEN,
						"/sys/class/uio/uio%d/maps/map%d/addr", uio, map);
			FILE *mapAddrFile = fopen(mapAddrPath, "r");
			if (!mapAddrFile) {
				continue;
			}

			fscanf(mapAddrFile, "%x", &mapAddr);
			if (mapAddr != addr) {
				continue;
			}

			*uioNum = uio;
			*mapNum = map;
			return;
		}
	}

	*uioNum = -1;
	*mapNum = -1;
}

/**
 * Initialize a ZMOD device by mapping the hardware memory area used
 * by it to a virtual memory area.
 *
 * @param addr the address of the ZMOD device
 * @param zmodInterrupt the interrupt number of the ZMOD device,
 *  unused on Linux platforms
 * @param fnZmodInterruptHandler a pointer to a function to be called
 *  when an interrupt occurs for the ZMOD device,
 *  unused on Linux platforms
 * @param zmodInterruptData a pointer to be passed back to the
 *  interrupt callback function, unused on Linux platforms.
 *
 * @return the address of the mapped virtual memory area
 */
uint32_t fnInitZmod(uintptr_t addr, int zmodInterrupt,
		void *fnZmodInterruptHandler, void *zmodInterruptData)
{
	int uioNum, mapNum;

	getDeviceByAddr(addr, &uioNum, &mapNum);
	if (uioNum < 0 || mapNum < 0) {
		fprintf(stderr,
				"That is not a valid UIO device or map number\n"
				"Check /sys/class/uio for more information about\n"
				"the available UIO devices\n");
		return -1;
	}

    UIO *uio = UIO_MAP(uioNum, mapNum);
    if (!uio) {
		fprintf(stderr, "Failed to map UIO device\n");
		return -1;
    }

    return (uintptr_t) uio->mapPtr;
}

/**
 * Destroy a ZMOD device by unmapping the virtual memory area
 * previously mapped fnInitZmod.
 *
 * @param addr the address of the virtual memory area previously
 *  mapped by fnInitZmod
 */
void fnDestroyZmod(uintptr_t addr)
{
	UIO_UNMAP((void *)addr);
}

/**
 * Write a value to a register in a virtual memory address
 * previously mapped by fnInitZmod.
 *
 * @param base_addr the address of the virtual memory address
 *  previously mapped by fnInitZmod
 * @param reg_addr the offset address of the register
 * @param val the value to write
 */
void fnWriteReg(uintptr_t base_addr, uint8_t reg_addr, uint32_t val)
{
	ACCESS_REG(base_addr, reg_addr) = val;
}

/**
 * Read a value from a register at a virtual memory address
 * previously mapped by fnInitZmod.
 * @param base_addr the address of the virtual memory address
 *  previously mapped by fnInitZmod
 * @param reg_addr the offset address of the register
 *
 * @return the value read
 */
uint32_t fnReadReg(uintptr_t base_addr, uint8_t reg_addr)
{
	return ACCESS_REG(base_addr, reg_addr);
}

#endif // LINUX_APP
