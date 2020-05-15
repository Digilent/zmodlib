/**
 * @file flash.h
 * @author Ciprian Hegbeli
 * @date 15 Nov 2019
 * @brief Function declarations used for Flash transfer.
 */

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "xparameters.h"
#include "xil_printf.h"
#ifdef PLATFORM_ZYNQ
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

uint32_t fnInitFlash(uintptr_t addr, uint16_t slave_addr);
void fnDestroyFlash(uintptr_t addr);
int fnReadFlash(uintptr_t addr, uint16_t data_addr, uint8_t *read_vals, size_t length);
int fnWriteFlash(uintptr_t addr, uint16_t data_addr, uint8_t *write_vals, size_t length);

#ifdef __cplusplus
}
#endif
#endif /* FLASH_H_ */
