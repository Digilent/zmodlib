#include <stdint.h>

#ifndef REG_H_
#define REG_H_

uint32_t fnInitZmod(uintptr_t addr, int zmodInterrupt,
		void *fnZmodInterruptHandler, void *zmodInterruptData);
void fnDestroyZmod(uintptr_t addr);

void fnWriteReg(uintptr_t base_addr, uint8_t reg_addr, uint32_t val);;
uint32_t fnReadReg(uintptr_t base_addr, uint8_t reg_addr);

#endif // REG_H_
