/**
 * @file zmoddac1411.h
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing function definitions of the ZMOD ADC1410 specific methods.
 */

#include "../Zmod/zmod.h"

#ifndef _ZMODADC1410_H
#define  _ZMODADC1410_H

#define ZMODADC1410_MAX_BUFFER_LEN	0x3FFF	// maximum buffer length supported by ZmodADC1410 IP


/**
 * ZMODADC1410 specific registers
 */

#define ZMODADC1410_REG_ADDR_TRIG			0x1C	///< TRIG				register address
#define ZMODADC1410_REG_ADDR_WINDOW			0x20	///< WINDOW			register address
#define ZMODADC1410_REG_ADDR_SC1LGMULTCOEF	0x24	///< SC1LGMULTCOEF 	register address
#define ZMODADC1410_REG_ADDR_SC1LGADDCOEF	0x28	///< SC1LGADDCOEF		register address
#define ZMODADC1410_REG_ADDR_SC1HGMULTCOEF	0x2C	///< SC1HGMULTCOEF	register address
#define ZMODADC1410_REG_ADDR_SC1HGADDCOEF	0x30	///< SC1HGADDCOEF 	register address
#define ZMODADC1410_REG_ADDR_SC2LGMULTCOEF	0x34	///< SC2LGMULTCOEF 	register address
#define ZMODADC1410_REG_ADDR_SC2LGADDCOEF	0x38	///< SC2LGADDCOEF 	register address
#define ZMODADC1410_REG_ADDR_SC2HGMULTCOEF	0x3C	///< SC2HGMULTCOEF 	register address
#define ZMODADC1410_REG_ADDR_SC2HGADDCOEF	0x40	///< SC2HGADDCOEF 	register address


/**
 * ZMODADC1410 specific register fields
 */
#define ZMODADC1410_REGFLD_CR_RUNSTOP 	ZMOD_REG_ADDR_CR,  4, 1					///< RUNSTOP 		field of CR register
#define ZMODADC1410_REGFLD_CR_TEST_MODE	ZMOD_REG_ADDR_CR,  5, 1					///< TEST_MODE	field of CR register
#define ZMODADC1410_REGFLD_SR_BUF_FULL		ZMOD_REG_ADDR_SR, 21, 1				///< BUF_FULL 	field of SR register
#define ZMODADC1410_REGFLD_IER_BUF_FULL		ZMOD_REG_ADDR_IER, 21, 1			///< BUF_FULL		field of IER register
#define ZMODADC1410_REGFLD_TRIG_CHANNEL		ZMODADC1410_REG_ADDR_TRIG, 0, 2		///< CHANNEL 		field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_MODE		ZMODADC1410_REG_ADDR_TRIG, 2, 2		///< MODE 		field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_EDGE		ZMODADC1410_REG_ADDR_TRIG, 4, 1		///< EDGE 		field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_LEVEL		ZMODADC1410_REG_ADDR_TRIG, 5, 14	///< LEVEL 		field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_SC1_AC_DC	ZMODADC1410_REG_ADDR_TRIG, 19, 1	///< SC1_AC_DC	field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_SC2_AC_DC	ZMODADC1410_REG_ADDR_TRIG, 20, 1	///< SC2_AC_DC 	field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_SC1_HG_LG	ZMODADC1410_REG_ADDR_TRIG, 21, 1	///< SC1_HG_LG 	field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_SC2_HG_LG	ZMODADC1410_REG_ADDR_TRIG, 22, 1	///< SC2_HG_LG 	field of TRIG register
#define ZMODADC1410_REGFLD_TRIG_SYNC		ZMODADC1410_REG_ADDR_TRIG, 23, 4	///< SYNC 		field of TRIG register
#define ZMODADC1410_REGFLD_WINDOW_WND		ZMODADC1410_REG_ADDR_WINDOW, 0, 26	///< WND field of WINDOW register
#define ZMODADC1410_REGFLD_SC1LGMULTCOEF_VAL	ZMODADC1410_REG_ADDR_SC1LGMULTCOEF, 0, 18	///< VAL field of SC1LGMULTCOEF register
#define ZMODADC1410_REGFLD_SC1HGMULTCOEF_VAL	ZMODADC1410_REG_ADDR_SC1HGMULTCOEF, 0, 18	///< VAL field of SC1HGMULTCOEF register
#define ZMODADC1410_REGFLD_SC1LGADDCOEF_VAL		ZMODADC1410_REG_ADDR_SC1LGADDCOEF, 0, 18	///< VAL field of SC1LGADDCOEF register
#define ZMODADC1410_REGFLD_SC1HGADDCOEF_VAL		ZMODADC1410_REG_ADDR_SC1HGADDCOEF, 0, 18	///< VAL field of SC1HGADDCOEF register
#define ZMODADC1410_REGFLD_SC2LGMULTCOEF_VAL	ZMODADC1410_REG_ADDR_SC2LGMULTCOEF, 0, 18	///< VAL field of SC2LGMULTCOEF register
#define ZMODADC1410_REGFLD_SC2HGMULTCOEF_VAL	ZMODADC1410_REG_ADDR_SC2HGMULTCOEF, 0, 18	///< VAL field of SC2HGMULTCOEF register
#define ZMODADC1410_REGFLD_SC2LGADDCOEF_VAL		ZMODADC1410_REG_ADDR_SC2LGADDCOEF, 0, 18	///< VAL field of SC2LGADDCOEF register
#define ZMODADC1410_REGFLD_SC2HGADDCOEF_VAL		ZMODADC1410_REG_ADDR_SC2HGADDCOEF, 0, 18	///< VAL field of SC2HGADDCOEF register

/**
 * ZMODADC1410 calibration
 */
#define ZMODADC1410_CALIB_USER_ADDR 0x7000 ///< Address in flash for user calibration area
#define ZMODADC1410_CALIB_FACT_ADDR 0x8100 ///< Address in flash for factory calibration area
#define ZMODADC1410_CALIB_ID	0xAD ///< Calibration ID

/**
 * Struct that maps the calibration data stored in the ZMODDAC1411 flash.
 * 128 bytes in size.
 */
typedef struct _CALIBECLYPSEADC {
    unsigned char   id; ///< 0xAD
    int             date; ///< unix time
    float           cal[2][2][2]; ///< [channel 0:1][low/high gain 0:1][0 multiplicative : 1 additive]
    unsigned char   nop[68]; ///< reserved
    char            lodg[22]; ///< BT log: Reference Board SN
    unsigned char   crc; ///< to generate: init 0 and -= 127 bytes; the checksum of the structure should be 0
} __attribute__((__packed__)) CALIBECLYPSEADC;

/**
 * Class containing functionality for ZMODADC1410.
 */
class ZMODADC1410: public ZMOD {
private:
	uint32_t *interruptBuffer;

	uint8_t acquirePolling(uint32_t* buffer, uint8_t channel, uint8_t mode, uint32_t level, uint32_t edge, uint32_t window, size_t length);
#ifndef LINUX_APP
	uint8_t acquireInterrupt(uint32_t* buffer, uint8_t channel, uint8_t mode, uint32_t level, uint32_t edge, uint32_t window, size_t length);
#endif //LINUX_APP

protected:

public:
	ZMODADC1410(uintptr_t baseAddress, uintptr_t dmaAddress, uintptr_t iicAddress, uintptr_t flashAddress, int zmodInterrupt, int dmaInterrupt);
	uint32_t* allocChannelsBuffer(size_t &length);
	void freeChannelsBuffer(uint32_t *buf, size_t length);
	uint16_t channelData(uint8_t channel, uint32_t data);
	int16_t signedChannelData(uint8_t channel, uint32_t data);

	void setTransferLength(size_t &length);
	void setTrigger(uint8_t channel, uint8_t mode, int16_t level, uint8_t edge, uint32_t window);
	void enableBufferFullInterrupt(uint8_t enBuffFullInt);
	uint8_t isBufferFull();
	void waitForBufferFullPolling();

	void start();
	void stop();

	uint8_t acquireTriggeredPolling(uint32_t* buffer, uint8_t channel, uint32_t level, uint32_t edge, uint32_t window, size_t length);
	uint8_t acquireImmediatePolling(uint32_t* buffer, uint8_t channel,  size_t &length);
#ifndef LINUX_APP
	uint8_t acquireTriggeredInterrupt(uint32_t* buffer, uint8_t channel, uint32_t level, uint32_t edge, uint32_t window, size_t length);
	uint8_t acquireImmediateInterrupt(uint32_t* buffer, uint8_t channel, size_t length);
#endif //LINUX_APP


	uint8_t autoTestRamp(uint8_t channel, uint32_t level, uint32_t edge, uint32_t window, size_t length);
	void processInterrupt() override;

	void setGain(uint8_t channel, uint8_t gain);
	void setCoupling(uint8_t channel, uint8_t coupling);
	int32_t computeCoefMult(float cg, uint8_t gain);
	int32_t computeCoefAdd(float ca, uint8_t gain);

	virtual int readUserCalib();
	void setCalibValues(uint8_t channel, uint8_t gain, float valG, float valA);

	float getVoltFromSignedRaw(int32_t raw, uint8_t gain);
};

#endif
