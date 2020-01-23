/**
 * @file flash.h
 * @author Ciprian Hegbeli
 * @date 15 Nov 2019
 * @brief Function declarations used for Flash transfer.
 */

#ifndef FLASH_H_
#define FLASH_H_

uint32_t fnInitFlash(uintptr_t addr, uint16_t slave_addr);
void fnDestroyFlash(uintptr_t addr);
int fnReadFlash(uintptr_t addr, uint16_t data_addr, uint8_t *read_vals, size_t length);
int fnWriteFlash(uintptr_t addr, uint16_t data_addr, uint8_t *write_vals, size_t length);

#endif /* FLASH_H_ */
