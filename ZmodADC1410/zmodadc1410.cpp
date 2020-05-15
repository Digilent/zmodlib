/**
 * @file zmodadc1410.cpp
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of the ZMOD ADC1410 specific methods.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "zmodadc1410.h"

/**
 * Initialize a ZMOD ADC1410 instance.
 *
 * @param baseAddress the base address of the ZMOD device,
 *  can be found by either looking in the device tree files on a Linux
 *  platform, or the xparameters.h file on a baremetal platform,
 *  or in Vivado's Address Editor tab.
 * @param dmaAddress the base address of the DMA device,
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
 * @param zmodInterrupt the interrupt number of the ZMOD device,
 *  can be found by looking at the xparameters.h file on a baremental
 *  platform, and is irrelevant on a Linux platform.
 * @param dmaInterrupt the interrupt number of the DMA device,
 *  can be found by looking at the xparameters.h file on a baremental
 *  platform, and is irrelevant on a Linux platform.
 */
ZMODADC1410::ZMODADC1410(uintptr_t baseAddress, uintptr_t dmaAddress, uintptr_t iicAddress, uintptr_t flashAddress, int zmodInterrupt, int dmaInterrupt)
		: ZMOD(baseAddress, dmaAddress, iicAddress, flashAddress, DMA_DIRECTION_RX, zmodInterrupt, dmaInterrupt)
{
	ZMOD::initCalib(sizeof(CALIBECLYPSEADC), ZMODADC1410_CALIB_ID, ZMODADC1410_CALIB_USER_ADDR, ZMODADC1410_CALIB_FACT_ADDR);
}

/**
* Allocates the data buffer used for AXI DMA transfers, 4 bytes for each element (sample).
* The length of the allocated buffer is limited to the maximum supported buffer length (0x3FFF),
* altering the value of the reference parameter accordingly.
*
* @param length the number of elements (samples) in the buffer - passed by reference
*
* @return the pointer to the allocated buffer
*
*/
uint32_t* ZMODADC1410::allocChannelsBuffer(size_t &length) {
	if(length > ZMODADC1410_MAX_BUFFER_LEN)
	{
		length = ZMODADC1410_MAX_BUFFER_LEN;
	}
	return (uint32_t *)ZMOD::allocDMABuffer(length * sizeof(uint32_t));
}

/**
* Free the data buffer used for AXI DMA transfers, 4 bytes for each sample.
*
 * @param buf the address of the DMA buffer
* @param length the number of samples in the buffer.
*
*
*/
void ZMODADC1410::freeChannelsBuffer(uint32_t *buf, size_t length) {
	ZMOD::freeDMABuffer(buf, length * sizeof(uint32_t));
}
/**
 * Set the trigger parameters of the data acquisition.
 *
 * @param channel the trigger channel,
 *  0 for channel 1, 1 for channel 2
 * @param mode the trigger mode for the data acquisition,
 *  0 for normal trigger, 1 for not trigger
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition
 *
 * @todo change level to a 16bit unsigned number when hardware changes
 */
void ZMODADC1410::setTrigger(uint8_t channel, uint8_t mode, int16_t level, uint8_t edge, uint32_t window)
{
	writeRegFld(ZMODADC1410_REGFLD_TRIG_CHANNEL, channel);
	writeRegFld(ZMODADC1410_REGFLD_TRIG_MODE, mode);
	writeSignedRegFld(ZMODADC1410_REGFLD_TRIG_LEVEL, level);
	writeRegFld(ZMODADC1410_REGFLD_TRIG_EDGE, edge);
    // Set Window position
    writeRegFld(ZMODADC1410_REGFLD_WINDOW_WND, window);
}

/**
 * Set the length of a transfer.
 *  The length of the allocated buffer is limited to the maximum supported buffer length (0x3FFF),
 * altering the value of the reference parameter accordingly.
 *
 * @param length the length of the transfer in number of elements (samples) to be
 *  transfered - passed by reference
 *
 */
void ZMODADC1410::setTransferLength(size_t &length) {
	if(length > ZMODADC1410_MAX_BUFFER_LEN)
	{
		length = ZMODADC1410_MAX_BUFFER_LEN;
	}
	// multiply by the size of the data
	setTransferSize(length * sizeof(uint32_t));
}

/**
 * Extract channel data from a buffer element.
 *
 * @param channel the channel to extract  0 for channel 1, 1 for channel 2
 * @param data the buffer element from which to extract the channel data (32 bits)
 *
 * @return the channel data (14 bits)
 */
uint16_t ZMODADC1410::channelData(uint8_t channel, uint32_t data)
{
	return (channel ? (data >> 2) : (data >> 18)) & 0x00003FFF;
}

/**
 * Extract signed channel data from a buffer element, making sure the most significant bit
 * is taken into account.
 *
 * @param channel the channel to extract  0 for channel 1, 1 for channel 2
 * @param data the buffer element from which to extract the channel data (32 bits)
 *
 * @return the signed channel data (14 bits)
 */
int16_t ZMODADC1410::signedChannelData(uint8_t channel, uint32_t data) {
	return toSigned(channelData(channel, data), 14);
}

/**
 * Acquire data using a polling method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate the
 *  requested acquisition
 * @param channel the trigger channel,
 *  0 for channel 1, 1 for channel 2
 * @param mode the trigger mode for the data acquisition,
 *  0 for normal trigger, 1 for not trigger
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition,
 *  can be any unsigned number smaller than length
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquirePolling(uint32_t* buffer, uint8_t channel, uint8_t mode, uint32_t level,
		uint32_t edge, uint32_t window, size_t length)
{
	int rc;

	// Set trigger data
    setTrigger(channel, mode, level, edge, window);

    // DMA RX transfer length in number of elements
    setTransferLength(length);

    // disable buffer full ZMODADC1410 IP interrupt
    enableBufferFullInterrupt(0);

    // RunStop bit = 1
	start();

	// waits until buffer full bit is set by the ZMODADC1410 IP
	waitForBufferFullPolling();

    // Start DMA Transfer
	rc = startDMATransfer(buffer);
	if (rc) {
		return ERR_FAIL;
	}

    // Wait for DMA to Complete transfer
    while(!isDMATransferComplete()) {}

    return ERR_SUCCESS;
}


/**
 * Check if the ZMODADC1410 buffer is full by reading the Buffer Full bit in Status Register.
 *
 * @return true if the DMA transfer completed, false if it is still running
 */
uint8_t ZMODADC1410::isBufferFull()
{
	return readRegFld(ZMODADC1410_REGFLD_SR_BUF_FULL);
}

/**
* Wait (by polling) until the ZMODADC1410 sets the Buffer Full bit in Status Register,
* will block until buffer full is set.
*
*/
void ZMODADC1410::waitForBufferFullPolling()
{
    // Wait for ZmodADC to fill internal buffer
    while(!isBufferFull()) {}

    // Clear Status Bit
    writeRegFld(ZMODADC1410_REGFLD_SR_BUF_FULL, 1);
}

/**
 * Enables / disables the buffer full ZMODADC1410 IP interrupt.
 *
 * @param enBuffFullInt the value for the buffer enable buffer full interrupt
 *  transfered
 */
void ZMODADC1410::enableBufferFullInterrupt(uint8_t enBuffFullInt)
{
	// Enable / Disable BUF_FULL flag on INTR_EN
	writeRegFld(ZMODADC1410_REGFLD_IER_BUF_FULL, enBuffFullInt);
}

/**
* Start the ADC to acquire data according to previous acquisition settings.
*
*/
void ZMODADC1410::start()
{
    // Clear Status Bit
    writeRegFld(ZMODADC1410_REGFLD_SR_BUF_FULL, 1);

    // Set RunStop Bit
	writeRegFld(ZMODADC1410_REGFLD_CR_RUNSTOP, 1);
}

/**
* Stop the ADC acquisition process.
*
*/
void ZMODADC1410::stop()
{
	writeRegFld(ZMODADC1410_REGFLD_CR_RUNSTOP, 0);
}
/**
 * Acquire data using a polling method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate
 *  the requested acquisition
 * @param channel the channel for which trigger is set: 0 for channel 1,
 *  1 for channel 2
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition,
 *  can be any unsigned number smaller than length
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquireTriggeredPolling(uint32_t* buffer, uint8_t channel, uint32_t level,
		uint32_t edge, uint32_t window, size_t length)
{
	return acquirePolling(buffer, channel, 0, level, edge, window, length);
}

/**
 * Acquire data using a polling method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate
 *  the requested acquisition
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquireImmediatePolling(uint32_t* buffer, uint8_t channel, size_t &length)
{
	return acquirePolling(buffer, channel, 1, 0, 0, 0, length);
}

#ifndef LINUX_APP
/**
 * (Baremetal only)
 * Acquire data using an interrupt method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate
 *  the requested acquisition
 * @param channel the channel for which trigger is set: 0 for channel 1, 1 for channel 2
 * @param mode the trigger mode for the data acquisition,
 *  0 for normal trigger, 1 for not trigger
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition,
 *  can be any unsigned number smaller than length
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquireInterrupt(uint32_t* buffer, uint8_t channel, uint8_t mode, uint32_t level,
		uint32_t edge, uint32_t window, size_t length)
{
	interruptBuffer = buffer;

	// Enable interrupts
	writeRegFld(ZMOD_REGFLD_CR_INTR_EN, 1);

	// Set trigger data
    setTrigger(channel, mode, level, edge, window);


    // DMA RX transfer length in number of elements
    setTransferLength(length);

    // Enable BUF_FULL flag on INTR_EN
    enableBufferFullInterrupt(1);

    // RunStop ZmodADC1410 bit = 1
    start();

    // Wait for DMA to Complete transfer
    while(!isDMATransferComplete()) {}

    return ERR_SUCCESS;
}

/**
 * (Baremetal only)
 * Acquire data using an interrupt method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate
 *  the requested acquisition
 * @param channel the channel for which trigger is set: 0 for channel 1,
 *  1 for channel 2
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition,
 *  can be any unsigned number smaller than length
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquireTriggeredInterrupt(uint32_t* buffer, uint8_t channel, uint32_t level,
		uint32_t edge, uint32_t window, size_t length)
{
	return acquireInterrupt(buffer, channel, 0, level, edge, window, length);
}

/**
 * (Baremetal only)
 * Acquire data using an interrupt method, will block until the acquisition
 * completes.
 *
 * @param buffer the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate
 *  the requested acquisition
 * @param channel the channel on which to run the data acquisition,
 *  0 for channel 1, 1 for channel 2s
 * @param length the number of samples to be acquired
 *
 * @return 0 on success, any other number on failure
 */
uint8_t ZMODADC1410::acquireImmediateInterrupt(uint32_t* buffer, uint8_t channel, size_t length)
{
	return acquireInterrupt(buffer, channel, 1, 0, 0, 0, length);
}

#endif //LINUX_APP

/**
 * Call when a ZMOD interrupt occurs.
 * For example when ZMOD's buffer full interrupt occurs, starts the DMA transfer.
 */
void ZMODADC1410::processInterrupt()
{
	if (readRegFld(ZMODADC1410_REGFLD_SR_BUF_FULL) == 1 &&
			readRegFld(ZMODADC1410_REGFLD_IER_BUF_FULL) == 1) {
	    // Start DMA Transfer
		startDMATransfer(interruptBuffer);

		// Clear interrupt status flag (acknowledge interrupt)
		writeRegFld(ZMODADC1410_REGFLD_SR_BUF_FULL, 1);
	}
}

/**
* Reads the calibration data into the calib class member.
* It calls the readUserCalib from the ZMOD class level to read the user calibration data
*  as an array of bytes into the area pointed by calib base class member.
* Then the calibration data is interpreted according to CALIBECLYPSEADC structure.
 * @return the status: ERR_SUCCESS for success, ERR_CALIB_ID for calib ID error,
 *  ERR_CALIB_CRC for CRC error, ERR_FAIL if calibration is not initialized.
*/
int ZMODADC1410::readUserCalib()
{
	int status;
	CALIBECLYPSEADC *pCalib;

	if(!calib)
	{
		return ERR_FAIL;
	}
	// read the user calibration data as an array of bytes, into the area pointed by calib base class member
	status = ZMOD::readUserCalib();
	if (status == ERR_SUCCESS)
	{
		// interpret the calib data as a CALIBECLYPSEADC data
		pCalib = (CALIBECLYPSEADC *)calib;

		// fill the calibration related registers
		// float           cal[2][2][2];   // [channel 0:1][low/high gain 0:1][0 multiplicative : 1 additive]
		writeRegFld(ZMODADC1410_REGFLD_SC1HGMULTCOEF_VAL, computeCoefMult(pCalib->cal[0][1][0], 1));
		writeRegFld(ZMODADC1410_REGFLD_SC1HGADDCOEF_VAL,   computeCoefAdd(pCalib->cal[0][1][1], 1));
		writeRegFld(ZMODADC1410_REGFLD_SC1LGMULTCOEF_VAL, computeCoefMult(pCalib->cal[0][0][0], 0));
		writeRegFld(ZMODADC1410_REGFLD_SC1LGADDCOEF_VAL,   computeCoefAdd(pCalib->cal[0][0][1], 0));

		writeRegFld(ZMODADC1410_REGFLD_SC2HGMULTCOEF_VAL, computeCoefMult(pCalib->cal[1][1][0], 1));
		writeRegFld(ZMODADC1410_REGFLD_SC2HGADDCOEF_VAL,   computeCoefAdd(pCalib->cal[1][1][1], 1));
		writeRegFld(ZMODADC1410_REGFLD_SC2LGMULTCOEF_VAL, computeCoefMult(pCalib->cal[1][0][0], 0));
		writeRegFld(ZMODADC1410_REGFLD_SC2LGADDCOEF_VAL,   computeCoefAdd(pCalib->cal[1][0][1], 0));

	}

	return ERR_SUCCESS;
}

/**
* Performs an auto test in ramp mode.
* It puts the Analog to Digital converter in test mode ramp mode, then performs an acquisition according
*  to the specified trigger parameters. Then it checks the acquired data to be consistent with the trigger parameters.
* @param channel the channel for which trigger is set: 0 for channel 1, 1 for channel 2
 * @param level the level on which to run the data acquisition,
 *  can be any valid 14bit unsigned number
 * @param edge the trigger edge at which to run the data acquisition,
 *  0 for rising edge, 1 for falling edge
 * @param window the window position at which to run the data acquisition,
 *  can be any unsigned number smaller than length
 * @param length the number of samples to be acquired
 * @return the status: ERR_SUCCESS for success, ERR_FAIL if the acquired data does not match the expected data.
*/
uint8_t ZMODADC1410::autoTestRamp(uint8_t channel, uint32_t level, uint32_t edge, uint32_t window, size_t length)
{
	uint32_t *buf;
	uint8_t status = ERR_SUCCESS;
	uint32_t val, expected;
	// put the ADC1410 in test mode
	writeRegFld(ZMODADC1410_REGFLD_CR_TEST_MODE, 1);

	// send commands to Zmod IC to configure it in ramp mode
	uint32_t sentCommands[] = {
		0x00000502, //Write CMD -> Select CHB (SC1)
		0x00001420, //Write CMD -> Configure Output mode register (CH1), 0x00001420 for binary offset
		0x00000D0F, //Write CMD -> Test Mode (Ramp)
		0x00000501, //Write CMD -> Select CHA (SC2)
		0x00001430, //Write CMD -> Configure Output mode register (CH2), 0x00001430 for binary offset
		0x00000D0F, //Write CMD -> Test Mode (Ramp)
	};

	size_t sentCommandsLength = sizeof(sentCommands) / sizeof(sentCommands[0]);

	sendCommands(sentCommands, sentCommandsLength);

	// wait for hardware reconfiguration
	usleep(1000);

	// allocate buffer
	buf = allocChannelsBuffer(length);

	// perform the acquisition
	acquireTriggeredPolling(buf, channel, level,	edge, window, length);

	// compare the acquired data vs expected data
	for (uint32_t i = 0; i < length; i++)
	{
		val = channelData(channel, buf[i]);
		expected = (level - window + i) & ((1<<14) - 1);
		if (val != expected)
		{
			status = ERR_FAIL;
		}
	}

	// remove the ADC1410 from the test mode
	writeRegFld(ZMODADC1410_REGFLD_CR_TEST_MODE, 0);

	// put adc in normal mode (remove ramp)
	uint32_t sentCommands1[] = {
		0x00000502, //Write CMD -> Select CHB (SC1)
		0x00001421, //Write CMD -> Configure Output mode register (CH1), 2's complement
		0x00000D00, //Write CMD -> Test Mode (Normal)
		0x00000501, //Write CMD -> Select CHA (SC2)
		0x00001431, //Write CMD -> Configure Output mode register (CH2), 2's complement
		0x00000D00, //Write CMD -> Test Mode (Normal)
	};

	size_t sentCommandsLength1 = sizeof(sentCommands1) / sizeof(sentCommands1[0]);

	sendCommands(sentCommands1, sentCommandsLength1);

	// free buffer
	freeChannelsBuffer(buf, length);

	// wait for hardware reconfiguration
	usleep(1000);

	return status;
}

/**
* Set the gain for a channel.
* @param channel 0 for channel 1, 1 for channel 2
* @param gain the gain : 0 for LOW gain, 1 for HIGH gain
*/
void ZMODADC1410::setGain(uint8_t channel, uint8_t gain)
{
	if(channel)
	{
		writeRegFld(ZMODADC1410_REGFLD_TRIG_SC2_HG_LG, gain);
	}
	else
	{
		writeRegFld(ZMODADC1410_REGFLD_TRIG_SC1_HG_LG, gain);
	}
}

/**
 * Set the coupling for a channel.
 * @param channel 0 for channel 1, 1 for channel 2
 * @param coupling 0 for DC Coupling, 1 for AC Coupling
 */
void ZMODADC1410::setCoupling(uint8_t channel, uint8_t coupling)
{
	if(channel)
	{
		writeRegFld(ZMODADC1410_REGFLD_TRIG_SC2_AC_DC, coupling);
	}
	else
	{
		writeRegFld(ZMODADC1410_REGFLD_TRIG_SC1_AC_DC, coupling);
	}
}

/**
 * Set a pair of calibration values for a specific channel and gain into the calib area (interpreted as CALIBECLYPSEADC).
 * In order for this change to be applied to user calibration area from flash, writeUserCalib function must be called.
 * @param channel the channel for which calibration values are set: 0 for channel 1, 1 for channel 2
 * @param gain the gain for which calibration values are set: 0 for LOW gain, 1 for HIGH gain
 * @param valG the gain calibration value to be set
 * @param valA the additive calibration value to be set
 */
void ZMODADC1410::setCalibValues(uint8_t channel, uint8_t gain, float valG, float valA)
{
	CALIBECLYPSEADC *pCalib;
	if(calib)
	{
		// interpret the calib data as a CALIBECLYPSEADC data
		pCalib = (CALIBECLYPSEADC *)calib;
		pCalib->cal[channel][gain][0] = valG;
		pCalib->cal[channel][gain][1] = valA;
	}
}

#define IDEAL_RANGE_ADC_HIGH 1.0 ///< Ideal range ADC LOW
#define IDEAL_RANGE_ADC_LOW 25.0 ///< Ideal range ADC HIGH
#define REAL_RANGE_ADC_HIGH 1.086 ///< Real range ADC HIGH
#define REAL_RANGE_ADC_LOW 26.25 ///< Real range ADC LOW

/**
 * Computes the Multiplicative calibration coefficient.
 * @param cg gain coefficient as it is stored in Flash
 * @param gain 0 LOW and 1 HIGH
 * @return a signed 32 value containing the multiplicative coefficient in the 18 lsb bits: bit 17 sign, bit 16-0 the value
 */
int32_t ZMODADC1410::computeCoefMult(float cg, uint8_t gain)
{
	float fval = (gain ? (REAL_RANGE_ADC_HIGH/IDEAL_RANGE_ADC_HIGH):(REAL_RANGE_ADC_LOW/IDEAL_RANGE_ADC_LOW))*(1 + cg)*(float)(1<<16);
	int32_t ival = (int32_t) (fval + 0.5);	// round
	ival &= (1<<18) - 1; // keep only 18 bits
	return ival;
}

/**
 * Computes the Additive calibration coefficient.
 * @param ca - add coefficient as it is stored in Flash
 * @param gain 0 LOW and 1 HIGH
 * @return a signed 32 value containing the additive coefficient in the 18 lsb bits: bit 17 sign, bit 16-0 the value
 */
int32_t ZMODADC1410::computeCoefAdd(float ca, uint8_t gain)
{
	float fval = ca / (gain ? IDEAL_RANGE_ADC_HIGH:IDEAL_RANGE_ADC_LOW)*(float)(1<<17);
	int32_t ival = (int32_t) (fval + 0.5);	// round
	ival &= (1<<18) - 1; // keep only 18 bits
	return ival;
}

/**
 * Converts a signed raw value (provided by ZmodADC1410 IP core) to a value in Volts measure unit.
 * @param raw - the signed value as .
 * @param gain 0 LOW and 1 HIGH
 * @return the Volts value.
 */
float ZMODADC1410::getVoltFromSignedRaw(int32_t raw, uint8_t gain)
{
	float vMax = gain ? IDEAL_RANGE_ADC_HIGH:IDEAL_RANGE_ADC_LOW;
	float fval = (float)raw * vMax / (float)(1<<13);
	return fval;
}


