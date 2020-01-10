/**
 * @file reg.c
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of the register writing functions.
 */

#ifndef LINUX_APP

#include <xil_io.h>
#include "../../reg.h"
#include "../intc/intc.h"

extern XScuGic sIntc;
extern bool fIntCInit;

/**
 * Initialize a ZMOD device by initializing its interrupts.
 *
 * @param addr the address of the ZMOD device
 * @param zmodInterrupt the interrupt number of the ZMOD device
 * @param fnZmodInterruptHandler a pointer to a function to be called
 *  when an interrupt occurs for the ZMOD device
 * @param zmodInterruptData a pointer to be passed back to the
 *  interrupt callback function
 *
 * @return the address of the ZMOD device, the same one already
 *  provided to this function, but returned from the convenience of
 *  commonization with the Linux implementation
 */
uint32_t fnInitZmod(uintptr_t addr, int zmodInterrupt,
		void *fnZmodInterruptHandler, void *zmodInterruptData)
{
	ivt_t ivt[] = {
		{ zmodInterrupt, (XInterruptHandler)fnZmodInterruptHandler, zmodInterruptData },
	};

	// Init interrupt controller
	if (!fIntCInit) {
		fnInitInterruptController(&sIntc);
		fIntCInit = true;
	}

	// Enable all interrupts in the interrupt vector table
	fnEnableInterrupts(&sIntc, &ivt[0], sizeof(ivt)/sizeof(ivt[0]));

	return addr;
}

/**
 * Destroy a ZMOD device.
 *
 * @param addr the address of the ZMOD device
 */
void fnDestroyZmod(uintptr_t addr)
{
}

/**
 * Write a value to a register of a ZMOD device.
 *
 * @param base_addr the address of the ZMOD device
 * @param reg_addr the offset address of the register
 * @param val the value to write
 */
void fnWriteReg(uintptr_t base_addr, uint8_t reg_addr, uint32_t val)
{
	Xil_Out32(base_addr + reg_addr, val);
}

/**
 * Read a value from a register of a ZMOD device.
 *
 * @param base_addr the address of the ZMOD device
 * @param reg_addr the offset address of the register
 *
 * @return the value read
 */
uint32_t fnReadReg(uintptr_t base_addr, uint8_t reg_addr)
{
	return Xil_In32(base_addr + reg_addr);
}

#endif // LINUX_APP
