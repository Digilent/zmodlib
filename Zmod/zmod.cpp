/**
 * @file zmod.cpp
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of the ZMOD general methods.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "zmod.h"
#include "flash.h"

void fnZmodInterruptHandler(void *data);

/**
 * ZMOD class constructor.
 *
 * @param baseAddr the base address of the ZMOD device,
 *  can be found by either looking in the device tree files on a Linux
 *  platform, or the xparameters.h file on a baremetal platform,
 *  or in Vivado's Address Editor tab.
 * @param dmaAddr the base address of the DMA device,
 *  can be found by either looking in the device tree files on a Linux
 *  platform, or the xparameters.h file on a baremetal platform,
 *  or in Vivado's Address Editor tab.
 * @param iicAddress the base address of the I2C device used for flash,
 *  can be found by either looking in the device tree files on a Linux
 *  platform, or the xparameters.h file on a baremetal platform,
 *  or in Vivado's Address Editor tab.
 * @param flashAddress is the I2C slave address of the I2C device used
 *  for flash, can be found in the carrier board reference manual,
 *  associated to the SZG connector where the ZMpd is plugged,
 * @param direction the direction of the DMA transfer,
 *  can be either DMA_DIRECTION_TX for a transfer from the processor
 *  to the FPGA, or DMA_DIRECTION_RX for a transfer from the FPGA
 *  to the processor.
 * @param zmodInterrupt the interrupt number of the ZMOD device,
 *  can be found by looking at the xparameters.h file on a baremental
 *  platform, and is irrelevant on a Linux platform.
 * @param dmaInterrupt the interrupt number of the DMA device,
 *  can be found by looking at the xparameters.h file on a baremental
 *  platform, and is irrelevant on a Linux platform.
 */
ZMOD::ZMOD(uintptr_t baseAddress, uintptr_t dmaAddress, uintptr_t iicAddress, uintptr_t flashAddress,
		enum dma_direction direction, int zmodInterrupt, int dmaInterrupt) {
	baseAddr = fnInitZmod(baseAddress,
			zmodInterrupt, (void *)fnZmodInterruptHandler, (void *)this);
	dmaAddr = fnInitDMA(dmaAddress, direction, dmaInterrupt);
	flashAddr = fnInitFlash(iicAddress, flashAddress);

	this->direction = direction;
	transferSize = 0;
	calib = 0;	// this will be later allocated by allocCalib

	// toggle reset bit
	writeRegFld(ZMOD_REGFLD_CR_RST, 1);
	writeRegFld(ZMOD_REGFLD_CR_RST, 0);
}

/**
* ZMOD class destructor. Destroys hardware instances.
*
*/
ZMOD::~ZMOD() {
	fnDestroyZmod(baseAddr);
	fnDestroyDMA(dmaAddr);
	fnDestroyFlash(flashAddr);
	if(calib)
	{
		free(calib);
	}
}

/**
 * Write a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 * @param value the value to write in the register
 *
 * This function writes a value to a Zmod IP register,
 *  specified by its offset address.
 */
void ZMOD::writeReg(uint8_t regAddr, uint32_t value) {
	fnWriteReg(baseAddr, regAddr, value);
}

/**
 * Read a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 *
 * @return the value read from the register
 *
 * This function returns the value read from a Zmod IP register,
 *  specified by its offset address.
 */
uint32_t ZMOD::readReg(uint8_t regAddr) {
	return fnReadReg(baseAddr, regAddr);
}

/**
 * Write a value to a Zmod IP register field.
 * A Zmod IP register field is a number of contiguous bits inside a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 * @param lsbBit the index of the first bit to write out of the register
 * @param noBits the number of bits to write
 * @param value the unsigned value to write in the register, will only be written to the
 *  bits starting at lsbBit (inclusive) and ending at lsbBit + noBits (exclusive)
 */
void ZMOD::writeRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits, uint32_t value) {
	uint32_t regMask = ((1 << noBits) - 1) << lsbBit;
	uint32_t regValue = readReg(regAddr);

	// align value to bit lsb_bit
	value <<= lsbBit;

	// mask out any bits outside specified field
	value &= regMask;

	// mask out bits corresponding to the specified field
	regValue &= ~regMask;

	// set the values for the field bits
	regValue |= value;

	writeReg(regAddr, regValue);
}

/**
 * Read a Zmod IP register field.
 * A Zmod IP register field is a number of contiguous bits inside a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 * @param lsbBit the index of the first bit to write out of the register
 * @param noBits the number of bits to write
 *
 * @return the value read from the register, will only be read from the bits
 *  starting at lsbBit (inclusive) and ending at lsbBit + noBits (exclusive)
 */
uint32_t ZMOD::readRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits) {
	uint32_t reg_mask = ((1 << noBits) - 1) << lsbBit;
	uint32_t value = readReg(regAddr);

	// mask out bits corresponding to the specified field
	value &= reg_mask;

	// align value to bit 0 (right justify)
	value >>= lsbBit;

	return value;
}

/**
 * Write a Zmod IP register field with the signed value.
 * A Zmod IP register field is a number of contiguous bits inside a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 * @param lsbBit the index of the first bit to write out of the register
 * @param noBits the number of bits to write
 * @param value the signed value to write in the register, will only be written to the
 *  bits starting at lsbBit (inclusive) and ending at lsbBit + noBits (exclusive)
 */
void ZMOD::writeSignedRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits, int32_t value) {
	writeRegFld(regAddr, lsbBit, noBits, (uint32_t)value);
}

/**
 * Read a Zmod IP register field, providing the signed value.
 * A Zmod IP register field is a number of contiguous bits inside a Zmod IP register.
 *
 * @param regAddr the offset address of the register
 * @param lsbBit the index of the first bit to write out of the register
 * @param noBits the number of bits to write
 *
 * @return the signed value read from the register, will only be read from the bits
 *  starting at lsbBit (inclusive) and ending at lsbBit + noBits (exclusive)
 */
int32_t ZMOD::readSignedRegFld(uint8_t regAddr, uint8_t lsbBit, uint8_t noBits) {
	uint32_t uValue = readRegFld(regAddr, lsbBit, noBits);
	return toSigned(uValue, noBits);
}
/**
 * Converts an unsigned value represented on a number of bits (the most lsb bits)
 *  to a signed value.
 *
 * @param value - the value containing on its most lsb bits the value that must
 *  be interpreted as signed.
 * @param noBits - the number of bits to be interpreted.
 */
int32_t ZMOD::toSigned(uint32_t value, uint8_t noBits) {
	// align value to bit 31 (left justify), to preserve sign
	value <<= 32 - noBits;

	int32_t sValue = (int32_t)value;

	// align value to bit 0 (right justify)
	sValue >>= (32 - noBits);

	return sValue;
}


/**
 * Call when a ZMOD interrupt occurs. Implemented on the child class to implement
 * child specific interrupts. On the ZMOD level it should contain potential
 * ZMOD common fields interrupts (none defined yet).
 */
void ZMOD::processInterrupt(){
}

/**
 * Allocate a DMA buffer.
 *
 * @param size the size of the DMA buffer, in bytes
 *
 * @return the address of the DMA buffer
 */
void* ZMOD::allocDMABuffer(size_t size) {
	return fnAllocBuffer(dmaAddr, size);
}

/**
 * Free a DMA buffer.
 *
 * @param buf the address of the DMA buffer
 * @param size the size of the DMA buffer
 */
void ZMOD::freeDMABuffer(uint32_t *buf, size_t size) {
	return fnFreeBuffer(dmaAddr, buf, size);
}

/**
 * Set the length (in bytes) of a transfer.
 *
 * @param length the length of the transfer in number of bytes to be
 *  transfered
 */
void ZMOD::setTransferSize(size_t size) {
	transferSize = size;
	if (direction == DMA_DIRECTION_RX) {
		writeRegFld(ZMOD_REGFLD_AXIS_S2MM_LENGTH_LENGTH, transferSize);
	} else {
		writeRegFld(ZMOD_REGFLD_AXIS_MM2S_LENGTH_LENGTH, transferSize);
	}

}

/**
 * Start a DMA transfer using the transfer length configured previously.
 *
 * @return 0 on success, any other number on failure
 */
int ZMOD::startDMATransfer(uint32_t* buffer) {
	// transfer length is not configured
	if (transferSize < 1) {
		return ERR_FAIL;
	}

	return fnOneWayDMATransfer(dmaAddr, buffer, transferSize);
}

/**
 * Check if the DMA transfer previously started has completed.
 *
 * @return true if the DMA transfer completed, false if it is still running
 */
bool ZMOD::isDMATransferComplete() {
	return fnIsDMATransferComplete(dmaAddr);
}

/**
 * Send a command to the underlying interface of the ZMOD.
 *
 * @param command the command to send
 */
void ZMOD::sendCommand(uint32_t command) {
	// write in the command FIFO
	writeReg(ZMOD_REG_ADDR_CMD_TX, command);
}

/**
 * Receive a command from the underlying interface of the ZMOD.
 *
 * @return the received command
 */
uint32_t ZMOD::receiveCommand() {
	// read from the command FIFO
	return readReg(ZMOD_REG_ADDR_CMD_RX);
}

/**
 * Send multiple commands to the underlying interface of the ZMOD.
 *
 * @param commands a pointer to an array of commands to send
 * @param length the length of the array
 */
void ZMOD::sendCommands(uint32_t *commands, size_t length) {
	if (!commands) {
		return;
	}

	// Write each command to the internal FIFO
	for (size_t i = 0; i < length; i++) {
		sendCommand(commands[i]);
	}

	// Push out the commands from the internal FIFO to the interfaces
	writeRegFld(ZMOD_REGFLD_CR_CMD_RUNSTP, 1);
	// Indicate that a read is happening to avoid flushing the internal FIFO
	writeRegFld(ZMOD_REGFLD_CR_CMD_READ_EN, 1);
	// Wait until the internal FIFO is empty
	while(!readRegFld(ZMOD_REGFLD_SR_CMD_TX_DONE)) {}
	writeRegFld(ZMOD_REGFLD_SR_CMD_TX_DONE, 1);
}

/**
 * Receive multiple commands from the underlying interface of the ZMOD.
 *
 * @param commands a pointer to an array to receive the commands in
 *
 * @return the number of received commands
 */
size_t ZMOD::receiveCommands(uint32_t *commands) {
	if (!commands) {
		return 0;
	}

	// Get the number of bytes present in the RX FIFO
	size_t length = readRegFld(ZMOD_REGFLD_SR_CMD_RX_COUNT);

	// Read each command from the internal FIFO
	for (size_t i = 0; i < length; i++) {
		commands[i] = receiveCommand();
	}

	return length;
}

/**
 * Send and receive multiple commands to and from the underlying
 * interface of the ZMOD.
 * @param sentCommands a pointer to an array of commands to send
 * @param receivedCommands a pointer to an array to receive the commands in
 * @param sentCommandsLength the length of the array
 */
size_t ZMOD::sendReceiveCommands(uint32_t *sentCommands, uint32_t *receivedCommands, size_t sentCommandsLength) {
	size_t receivedCommandsLength;

	sendCommands(sentCommands, sentCommandsLength);
	receivedCommandsLength = receiveCommands(receivedCommands);

	return receivedCommandsLength;
}

/**
 * Reads the calibration data into the bytes array pointed by calib class member.
 * At the ZMOD class level, the calib area is regarded just as an array of bytes.
 * The area pointed by calib must be already allocated by calling initCalib
 *  (normally called from the child class).
 * This function expects and verifies that the first byte of the data retrieved
 * from flash is the calib id, while the last byte is the checksum.
 * @return the status: ERR_SUCCESS for success, ERR_CALIB_ID for calib ID error,
 *  ERR_CALIB_CRC for CRC error.
 */
int ZMOD::readUserCalib()
{
	int status;
	uint8_t crc;
	// read the user calibration data as an array of bytes
	status = fnReadFlash(flashAddr, userCalibAddr, calib, calibSize);
	if(status == ERR_SUCCESS)
	{
		// check the expected Calib ID against the first byte of the calib area
		if(calib[0] != calibID)
		{
			// Calib ID error
			status = ERR_CALIB_ID;
		}
		else
		{
			// compute the CRC and check against the last byte of the calib area
			crc = computeCRC(calib, calibSize - 1);
			if(crc != calib[calibSize-1])
			{
				// CRC error
				status = ERR_CALIB_CRC;
			}
		}
	}
	return status;
}

/**
 * Initialize the calibration related data.
 * It allocates the calib area as an array of bytes having the desired length.
 * It also stores the calibration ID and the user and factory calibration flash addresses.
 * In the end it reads the user calibration values by calling the Zmod specific
 *  (virtual) readUserCalib function.
 * @param calibSize the size of calibration data to be retrieved from Flash.
 * @param calibID the ID of the calibration data.
 * @param userCalibAddr the flash address of user calibration area.
 * @param factCalibAddr the flash address of factory calibration area.
 * @return the status: ERR_SUCCESS for success, ERR_FAIL for allocation error.
 */
int ZMOD::initCalib(uint32_t calibSize, uint8_t calibID, uint32_t userCalibAddr, uint32_t factCalibAddr)
{
	calib = (uint8_t *)malloc(calibSize);
	this->calibSize = calibSize;
	this->calibID = calibID;
	this->userCalibAddr = userCalibAddr;
	this->factCalibAddr = factCalibAddr;
	readUserCalib();
	return ((calib != 0) ? ERR_SUCCESS: ERR_FAIL);
}

/**
 * Writes the calibration data from the area pointed by calib member into the user calibration area.
 * At the ZMOD class level, the calib area is regarded just as an array of bytes.
 * The area pointed by calib must be already allocated by calling initCalib
 * (normally called from the child class).
 * Before writing it fills the first byte of the calib area with the calibration ID and
 * the last byte of the calib area with the checksum of all but the last byte of the calib area.
 *
 */
void ZMOD::writeUserCalib()
{
	uint8_t crc;
	// fill the calib ID byte on the first byte of calib
	calib[0] = calibID;
	// compute the checksum on all but the last byte of the calib area
	crc = computeCRC(calib, calibSize - 1);
	// fill the checksum byte on the last byte of calib area
	calib[calibSize] = crc;

	fnWriteFlash(flashAddr, userCalibAddr, calib, calibSize);
}

/**
 * Restores the factory calibration data by reading it from factory calibration area into the
 *  area pointed by calib and writing it into the user calibration area.
 *
 */
int ZMOD::restoreFactoryCalib()
{
	// reads the factory calibration into calib as an array of bytes
	int status;
	// read the user calibration data as an array of bytes
	status = fnReadFlash(flashAddr, factCalibAddr, calib, calibSize);
	if(status == ERR_SUCCESS)
	{
		// writes the calibration data from the area pointed by calib member into the user calibration area.
		writeUserCalib();

		// this will generate calibration coefficients
		readUserCalib();
	}
	return status;
}

/**
 * Computes a one byte checksum of an array of bytes.
 * @param pData a pointer to an array of bytes
 * @param len the length of the array
 */
uint8_t ZMOD::computeCRC(uint8_t *pData, uint32_t len)
{
	uint8_t s = 0;
	uint32_t i;
	for(i = 0; i < len; i++)
	{
		s-= pData[i];
	}
	return s;
}

/**
 * Interrupt handler called when IP interrupt occurs.
 * @param data the callback data, set to be the pointer to the ZMOD object.
 */
void fnZmodInterruptHandler(void *data) {
	ZMOD *zmod = (ZMOD *)data;
	zmod->processInterrupt();
}


/**
 * Formats a float value in a string, with (eventually) sign, integer part, (eventually) decimal point and 3 decimals and (eventually) measure unit.
 * @param dest - the string to receive the formatted value. Must be previously allocated large enough (13 chars)
 * @param val -  the value to be formatted
 * @param unit - the string containing the measure unit. If NULL, no measure unit is added.
 */
void ZMOD::formatValue(char *dest, float val, const char *unit)
{
	if (val < 0)
	{
		val = -val; // make it positive
		dest[0] = '-';
		dest ++;
	}
	// round the 3'th decimal digit (the rest of the digits are decimated)
	val += 0.0005;
	int poz = (int) val;
	float f = val - (float)poz;
	if(f != 0.0)
	{
		sprintf(dest, "%d.%03d", poz, (int)(f*1000.0));
		// trim ending 0's
		int i = strlen(dest) - 1;
		while(dest[i] == '0')
		{
			dest[i--] = 0;
		}
		if(dest[i] == '.')
		{
			dest[i] = 0;
		}
	}
	else
	{
		sprintf(dest, "%d", poz);
	}

	if(unit)
	{
		strcat(dest, " ");
		strcat(dest, unit);
	}
}
