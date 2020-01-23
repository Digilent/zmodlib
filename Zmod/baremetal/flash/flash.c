/**
 * @file baremetal/flash/flash.c
 * @author Ciprian Hegbeli
 * @date 15 Nov 2019
 * @brief Function declarations used for Flash transfer.
 */

#ifndef LINUX_APP

#include <stdio.h>
#include <stdlib.h>

#include "xparameters.h"
#include "xil_printf.h"
#include "xiicps.h"
#include "sleep.h"
#include "xstatus.h"

#define IIC_SCLK_RATE 		400000 ///< I2C clock rates
#define FLASH_MAX_LENGTH	256 ///< Maximum flash transfer length

/**
 * Struct containing data specific to this flash instance.
 */
typedef struct _flash_env {
	uint16_t slave_addr; ///< Slave address
} FlashEnv;

extern XIicPs_Config XIicPs_ConfigTable[XPAR_XIICPS_NUM_INSTANCES]; ///< Instances of Iic devices supported

XIicPs XIicPS; ///< Iic device instance
bool fIicPSInit = false; ///< Iic device instance initialization flag

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
 * @param data_addr address within the flash
 * @param read_vals pointer to the Iic read values
 * @param length of the Iic transfer in bytes
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
 * @param data_addr address within the flash
 * @param write_vals pointer to the Iic write values
 * @param length of the Iic transfer in bytes
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


