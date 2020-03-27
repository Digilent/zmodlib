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
#ifdef __ZYNQ__
#include "xiicps.h"
#define Iic_Config XIicPs_Config
#define Iic_ConfigTable XIicPs_ConfigTable
#define Iic XIicPs
#define NUMINSTANCES XPAR_XIICPS_NUM_INSTANCES
#define Iic_CfgInitialize XIicPs_CfgInitialize
#else
#include "xiic.h"
#define Iic_Config XIic_Config
#define Iic_ConfigTable XIic_ConfigTable
#define Iic XIic
#define NUMINSTANCES XPAR_XIIC_NUM_INSTANCES
#define Iic_CfgInitialize XIic_CfgInitialize
#endif

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

extern Iic_Config Iic_ConfigTable[NUMINSTANCES];

Iic IicDev;				/* Instance of the IIC Device */
bool fIicInit = false; 	/* Initialization flag*/

/**
 * Extracting the configuration of Iic from the base address .
 *
 * @param Baseaddr the base address of the Iic device
 *
 * @return configuration pointer
 */
Iic_Config *Iic_LookupConfigBaseAddr(uintptr_t Baseaddr)
{
	Iic_Config *CfgPtr = NULL;
	s32 Index;

	for (Index = 0; Index < NUMINSTANCES; Index++) {
		if (Iic_ConfigTable[Index].BaseAddress == Baseaddr) {
			CfgPtr = &Iic_ConfigTable[Index];
			break;
		}
	}

	return CfgPtr;
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
	Iic_Config *pCfgPtr;
	int Status;

	FlashEnv *flash_env = (FlashEnv *)malloc(sizeof(FlashEnv));
	if (flash_env < 0) {
		return XST_FAILURE;
	}

	flash_env->slave_addr = slave_addr;

	//avoid multiple initializations
	if (fIicInit)
		goto iic_init_done;

	//extract configuration pointer
	pCfgPtr = Iic_LookupConfigBaseAddr(addr);
	if (!pCfgPtr) {
		xil_printf("No config found for %X\n", addr);
		return -1;
	}

	//apply configuration to the Iic pointer
	Status = Iic_CfgInitialize(&IicDev, pCfgPtr, pCfgPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %X\n");
		return -1;
	}

	//set the desired clock rate
#ifdef __ZYNQ__
	XIicPs_SetSClk(&XIicPS, IIC_SCLK_RATE);
#endif

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

#ifdef __ZYNQ__
	// Send the read address
	u8BytesSent = XIicPs_MasterSendPolled(&XIicPS, u8TxData, 2, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}

	// Receive function form the flash
	u8BytesSent = XIicPs_MasterRecvPolled(&XIicPS, (uint8_t *)read_vals, length, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}

#else
	u8BytesSent = XIic_Send(IicDev.BaseAddress, flash_env->slave_addr, u8TxData, 2, XIIC_STOP);

	u8BytesSent = XIic_Recv(IicDev.BaseAddress, flash_env->slave_addr, read_vals, length, XIIC_STOP);
#endif
	if (u8BytesSent < 0)
		return XST_FAILURE;
	else
		return XST_SUCCESS;

}

/**
 * Write to the Flash .
 *
 * @param addr is the Iic address of slave flash device
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

#ifdef __ZYNQ__
	// Send the data to the flash
	u8BytesSent = XIicPs_MasterSendPolled(&XIicPS, u8TxData, length + 2, flash_env->slave_addr);
	while (XIicPs_BusIsBusy(&XIicPS)) {}
#else
	u8BytesSent = XIic_Send(IicDev.BaseAddress, flash_env->slave_addr, u8TxData, length+2, XIIC_STOP);
#endif

	return (int)u8BytesSent;
}

#endif // LINUX_APP


