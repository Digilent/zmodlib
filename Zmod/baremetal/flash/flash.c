/******************************************************************************
 * @file Flash.c
 * Function declarations used for Flash transfer.
 *
 * @authors Ciprian Hegbeli
 *
 * @date 2015-Jan-20
 *
 * @copyright
 * (c) 2015 Copyright Digilent Incorporated
 * All Rights Reserved
 *
 * This program is free software; distributed under the terms of BSD 3-clause
 * license ("Revised BSD License", "New BSD License", or "Modified BSD License")
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name(s) of the above-listed copyright holder(s) nor the names
 *    of its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * @desciption
 *
 * @note
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who             Date        Changes
 * ----- --------------- ----------- --------------------------------------------
 *
 * </pre>
 *
 *****************************************************************************/

#ifndef LINUX_APP

#include <stdio.h>
#include <stdlib.h>

#include "xparameters.h"
#include "xil_printf.h"
#include "xiicps.h"
#include "sleep.h"
#include "xstatus.h"

//I2c clock rates
#define IIC_SCLK_RATE 		400000
//maximum flash transfer length
#define FLASH_MAX_LENGTH	256

//Flash environment structure
typedef struct _flash_env {
	uint16_t slave_addr;
} FlashEnv;

extern XIicPs_Config XIicPs_ConfigTable[XPAR_XIICPS_NUM_INSTANCES];

XIicPs XIicPS;				/* Instance of the IIC Device */
bool fIicPSInit = false; 	/* Initialization flag*/

/**
 * Extracting the configuration of Iic from the base address .
 *
 * @param Baseaddr the base address of the Iic device
 *
 * @return configuration pointer
 */
XIicPs_Config *XIicPs_LookupConfigBaseAddr(uintptr_t Baseaddr)
{
	XIicPs_Config *CfgPtr = NULL;
	s32 Index;

	for (Index = 0; Index < XPAR_XIICPS_NUM_INSTANCES; Index++) {
		if (XIicPs_ConfigTable[Index].BaseAddress == Baseaddr) {
			CfgPtr = &XIicPs_ConfigTable[Index];
			break;
		}
	}

	return (XIicPs_Config *)CfgPtr;
}

/**
 * Initialize the Iic Flash .
 *
 * @param addr the base address of the Iic device
 * @param slave_addr the Iic address of the Zmod
 *
 * @return XST_SUCCESS if successful
 */
uint32_t fnInitFlash(uintptr_t addr, uint16_t slave_addr)
{
	XIicPs_Config *pCfgPtr;
	int Status;

	FlashEnv *flash_env = (FlashEnv *)malloc(sizeof(FlashEnv));
	if (flash_env < 0) {
		return XST_FAILURE;
	}

	flash_env->slave_addr = slave_addr;

	//avoid multiple initializations
	if (fIicPSInit)
		goto iic_init_done;

	//extract configuration pointer
	pCfgPtr = XIicPs_LookupConfigBaseAddr(addr);
	if (!pCfgPtr) {
		xil_printf("No config found for %X\n", addr);
		return -1;
	}

	//apply configuration to the Iic pointer
	Status = XIicPs_CfgInitialize(&XIicPS, pCfgPtr, pCfgPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %X\n");
		return -1;
	}

	//set the desired clock rate
	XIicPs_SetSClk(&XIicPS, IIC_SCLK_RATE);

iic_init_done:
	return (uint32_t)flash_env;
}

/**
 * Destroy a Flash device.
 *
 * @param addr the address of the Flash device
 */
void fnDestroyFlash(uintptr_t addr)
{
	FlashEnv *flash_env = (FlashEnv *)addr;
	if (!flash_env)
		return;

	free(flash_env);
}

/**
 * Prepares the first bites to be transfered for both read
 * and write
 *
 * @param data pointer which will be transfered
 * @param data_addr the address of the device register
 */
void fnFormatAddr(uint8_t *data, uint16_t data_addr) {
	data[0] = (data_addr >> 8);
	data[1] = (data_addr & 0xff);
}

/**
 * Read fom Flash .
 *
 * @param addr is the Iic address of slave flash device
 * @param flash_addr address within the flash
 * @param length of the Iic transfer in bytes
 * @param read_vals pointer to the Iic read values
 *
 * @return XST_SUCCESS if successful
 */
int fnReadFlash(uintptr_t addr, uint16_t data_addr, uint8_t *read_vals, size_t length)
{
	uint8_t u8BytesSent;
	uint8_t u8TxData[2];

	FlashEnv *flash_env = (FlashEnv *)addr;
	if (!flash_env)
		return XST_FAILURE;

	fnFormatAddr(u8TxData, data_addr);

	// Send the read address
	u8BytesSent = XIicPs_MasterSendPolled(&XIicPS, u8TxData, 2, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}

	// Receive function form the flash
	u8BytesSent = XIicPs_MasterRecvPolled(&XIicPS, (uint8_t *)read_vals, length, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}

	if (u8BytesSent < 0)
		return XST_FAILURE;
	else
		return XST_SUCCESS;

}

/**
 * Write to the Flash .
 *
 * @param addr is the Iic address of slave flash device
 * @param flash_addr address within the flash
 * @param length of the Iic transfer in bytes
 * @param write_vals pointer to the Iic write values
 *
 * @return XST_SUCCESS if successful
 */
int fnWriteFlash(uintptr_t addr, uint16_t data_addr, uint8_t *write_vals, size_t length)
{
	uint8_t u8BytesSent;
	uint8_t u8TxData[FLASH_MAX_LENGTH];

	FlashEnv *flash_env = (FlashEnv *)addr;
	if (!flash_env)
		return XST_FAILURE;

	fnFormatAddr(u8TxData, data_addr);

	// Copy the data in to the send data structure
	memcpy(u8TxData + 2, (const void *)write_vals, length);

	// Send the data to the flash
	u8BytesSent = XIicPs_MasterSendPolled(&XIicPS, u8TxData, length + 2, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}

	return (int)u8BytesSent;
}

#endif // LINUX_APP


