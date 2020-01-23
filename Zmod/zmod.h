/**
 * @file zmod.h
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of the ZMOD general methods.
 */

#include <stdint.h>

#include "dma.h"
#include "reg.h"

#ifndef _ZMOD_H
#define  _ZMOD_H

/**
 * Error codes
 */
#define ERR_SUCCESS 	0 	///< Success
#define ERR_FAIL		-1	///< General fail
#define ERR_CALIB_CRC	-2	///< CRC check fail
#define ERR_CALIB_ID	-3	///< Calib ID check fail

/**
 * ZMOD common registers
 */
#define ZMOD_REG_ADDR_CR 				0x00	///< CR					register address
#define ZMOD_REG_ADDR_SR 				0x04	///< SR 				register address
#define ZMOD_REG_ADDR_IER	 			0x08	///< IER 				register address
#define ZMOD_REG_ADDR_CMD_TX 			0x0C	///< CMD_TX 			register address
#define ZMOD_REG_ADDR_CMD_RX 			0x10	///< CMD_RX 			register address
#define ZMOD_REG_ADDR_AXIS_S2MM_LENGTH 	0x14	///< AXIS_S2MM_LENGTH	register address
#define ZMOD_REG_ADDR_AXIS_MM2S_LENGTH 	0x18	///< AXIS_MM2S_LENGTH	register address
#define ZMOD_REGFLD_CR_CMD_RUNSTP		ZMOD_REG_ADDR_CR,  1, 1	///< CMD_RUNSTP 	field of CR register
#define ZMOD_REGFLD_CR_CMD_READ_EN  	ZMOD_REG_ADDR_CR,  2, 1	///< CMD_READ_EN 	field of CR register
#define ZMOD_REGFLD_CR_INTR_EN 			ZMOD_REG_ADDR_CR,  3, 1	///< INTR_EN 		field of CR register
#define ZMOD_REGFLD_CR_RST 				ZMOD_REG_ADDR_CR, 31, 1	///< RST 			field of CR register
#define ZMOD_REGFLD_SR_CMD_TX_DONE 		ZMOD_REG_ADDR_SR, 0, 1	///< TX_DONE 		field of SR register
#define ZMOD_REGFLD_SR_CMD_RUNNING  	ZMOD_REG_ADDR_SR, 2, 1	///< CMD_RUNNING	field of SR register
#define ZMOD_REGFLD_SR_CMD_TX_COUNT		ZMOD_REG_ADDR_SR, 3, 7	///< CMD_TX_COUNT	field of SR register
#define ZMOD_REGFLD_SR_CMD_RX_COUNT		ZMOD_REG_ADDR_SR, 10, 7	///< CMD_RX_COUNT	field of SR register
#define ZMOD_REGFLD_IER_CMD_TX_DONE		ZMOD_REG_ADDR_IER, 0, 1	///< CMD_TX_DONE		field of IER register
#define ZMOD_REGFLD_IER_CMD_RX_DONE		ZMOD_REG_ADDR_IER, 1, 1	///< CMD_RX_DONE  	field of IER register
#define ZMOD_REGFLD_IER_CMD_TX_RX_ERROR	ZMOD_REG_ADDR_IER, 2, 4	///< CMD_TX_RX_ERROR	field of IER register
#define ZMOD_REGFLD_AXIS_S2MM_LENGTH_LENGTH	ZMOD_REG_ADDR_AXIS_S2MM_LENGTH, 0, 26	///< LENGTH field of AXIS_S2MM_LENGTH register
#define ZMOD_REGFLD_AXIS_MM2S_LENGTH_LENGTH ZMOD_REG_ADDR_AXIS_MM2S_LENGTH, 0, 26	///< LENGTH field of AXIS_MM2S_LENGTH register

/**
 * Class containing functionality common to all ZMODs.
 */
class ZMOD {
protected:
	uintptr_t 	baseAddr; ///< Zmod base address
	uintptr_t 	dmaAddr; ///< DMA environment pointer
	uintptr_t 	flashAddr; ///< Flash  base address
	size_t	transferSize; ///< DMA transfer size
	uint8_t *calib; ///< pointer to calibration data
	uint32_t calibSize; ///< calibration size
	uint16_t userCalibAddr; ///< address of user calibration area
	uint16_t factCalibAddr;///< address of factory calibration area
	uint8_t calibID; ///< calibration ID
	enum dma_direction direction; ///< DMA tranfer direction
	int initCalib(uint32_t calibSize, uint8_t calibID, uint32_t userCalibAddr,uint32_t factCalibAddr);
	void* allocDMABuffer(size_t size);
	void freeDMABuffer(uint32_t *buf, size_t size);
	uint8_t computeCRC(uint8_t *pData, uint32_t len);

public:
	ZMOD(uintptr_t baseAddress, uintptr_t dmaAddress, uintptr_t iicAddress, uintptr_t flashAddress,
			enum dma_direction direction, int zmodInterrupt, int dmaInterrupt);
	~ZMOD();

	void writeReg(uint8_t regAddr, uint32_t value);
	void writeRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits, uint32_t value);
	uint32_t readReg(uint8_t regAddr);
	uint32_t readRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits);

	void writeSignedRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits, int32_t value);
	int32_t readSignedRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits);

	void setTransferSize(size_t size);
	int startDMATransfer(uint32_t* buffer);
	bool isDMATransferComplete();

	void sendCommand(uint32_t command);
	uint32_t receiveCommand();

	void sendCommands(uint32_t *commands, size_t length);
	size_t receiveCommands(uint32_t *commands);
	size_t sendReceiveCommands(uint32_t *sentCommands, uint32_t *receivedCommands, size_t length);

	virtual void processInterrupt();
	virtual int readUserCalib();
	void writeUserCalib();
	int restoreFactoryCalib();

	void formatValue(char *dest, float val, const char *unit);

protected:
	int32_t toSigned(uint32_t value, uint8_t noBits);
};

#endif
