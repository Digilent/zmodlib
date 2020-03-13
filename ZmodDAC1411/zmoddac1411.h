/**
 * @file zmodadc1410.h
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing function definitions of the ZMOD DAC1411 specific methods.
 */

#include "../Zmod/zmod.h"

#ifndef _ZMODDAC1411_H
#define  _ZMODDAC1411_H

#define ZmodDAC1411_MAX_BUFFER_LEN	0x3FFF	// maximum buffer length supported by ZmodDAC1411 IP


/**
 * ZMODDAC1411 specific register fields
 */
#define ZMODDAC1411_REG_ADDR_TRIG			0x1C	///< TRIG				register address
#define ZMODDAC1411_REG_ADDR_SC1LGMULTCOEF	0x24	///< SC1LGMULTCOEF 	register address
#define ZMODDAC1411_REG_ADDR_SC1LGADDCOEF	0x28	///< SC1LGADDCOEF		register address
#define ZMODDAC1411_REG_ADDR_SC1HGMULTCOEF	0x2C	///< SC1HGMULTCOEF	register address
#define ZMODDAC1411_REG_ADDR_SC1HGADDCOEF	0x30	///< SC1HGADDCOEF 	register address
#define ZMODDAC1411_REG_ADDR_SC2LGMULTCOEF	0x34	///< SC2LGMULTCOEF 	register address
#define ZMODDAC1411_REG_ADDR_SC2LGADDCOEF	0x38	///< SC2LGADDCOEF 	register address
#define ZMODDAC1411_REG_ADDR_SC2HGMULTCOEF	0x3C	///< SC2HGMULTCOEF 	register address
#define ZMODDAC1411_REG_ADDR_SC2HGADDCOEF	0x40	///< SC2HGADDCOEF 	register address

/**
 * ZMODDAC1411 specific register fields
 */
#define ZMODDAC1411_REGFLD_CR_DAC_EN 			ZMOD_REG_ADDR_CR,  4,  1	///< DAC_EN  			field of CR register
#define ZMODDAC1411_REGFLD_CR_TEST_MODE			ZMOD_REG_ADDR_CR,  5,  1	///< TEST_MODE	field of CR register
#define ZMODDAC1411_REGFLD_CR_OUT_ADDR_CNT_RST	ZMOD_REG_ADDR_CR,  6,  1	///< OUT_ADDR_CNT_RST	field of CR register
#define ZMODDAC1411_REGFLD_CR_DIV_RATE  		ZMOD_REG_ADDR_CR, 16, 14	///< DIV_RATE 		field of CR register
#define ZMODDAC1411_REGFLD_SR_BUF_FULL			ZMOD_REG_ADDR_SR, 21,  1	///< BUF_FULL 		field of SR register
#define ZMODDAC1411_REGFLD_IER_BUF_FULL			ZMOD_REG_ADDR_IER, 21, 1	///< BUF_FULL 		field of IER register
#define ZMODDAC1411_REGFLD_TRIG_SC1_HG_LG		ZMODDAC1411_REG_ADDR_TRIG, 21, 1	///< SC1_HG_LG 	field of TRIG register
#define ZMODDAC1411_REGFLD_TRIG_SC2_HG_LG		ZMODDAC1411_REG_ADDR_TRIG, 22, 1	///< SC2_HG_LG 	field of TRIG register
#define ZMODDAC1411_REGFLD_SC1LGMULTCOEF_VAL	ZMODDAC1411_REG_ADDR_SC1LGMULTCOEF, 0, 18	///< VAL field of SC1LGMULTCOEF register
#define ZMODDAC1411_REGFLD_SC1HGMULTCOEF_VAL	ZMODDAC1411_REG_ADDR_SC1HGMULTCOEF, 0, 18	///< VAL field of SC1HGMULTCOEF register
#define ZMODDAC1411_REGFLD_SC1LGADDCOEF_VAL		ZMODDAC1411_REG_ADDR_SC1LGADDCOEF, 0, 18	///< VAL field of SC1LGADDCOEF register
#define ZMODDAC1411_REGFLD_SC1HGADDCOEF_VAL		ZMODDAC1411_REG_ADDR_SC1HGADDCOEF, 0, 18	///< VAL field of SC1HGADDCOEF register
#define ZMODDAC1411_REGFLD_SC2LGMULTCOEF_VAL	ZMODDAC1411_REG_ADDR_SC2LGMULTCOEF, 0, 18	///< VAL field of SC2LGMULTCOEF register
#define ZMODDAC1411_REGFLD_SC2HGMULTCOEF_VAL	ZMODDAC1411_REG_ADDR_SC2HGMULTCOEF, 0, 18	///< VAL field of SC2HGMULTCOEF register
#define ZMODDAC1411_REGFLD_SC2LGADDCOEF_VAL		ZMODDAC1411_REG_ADDR_SC2LGADDCOEF, 0, 18	///< VAL field of SC2LGADDCOEF register
#define ZMODDAC1411_REGFLD_SC2HGADDCOEF_VAL		ZMODDAC1411_REG_ADDR_SC2HGADDCOEF, 0, 18	///< VAL field of SC2HGADDCOEF register

/**
 * ZMODDAC1411 calibration
 */
#define ZMODDAC1411_CALIB_USER_ADDR 0x7000 ///< Address in flash for user calibration area
#define ZMODDAC1411_CALIB_FACT_ADDR 0x8100 ///< Address in flash for factory calibration area
#define ZMODDAC1411_CALIB_ID	0xDA ///< Calibration ID

/**
 * Struct that maps the calibration data stored in the ZMODADC1410 flash.
 * 128 bytes in size.
 */
typedef struct _CALIBECLYPSEDAC {
    unsigned char   id; ///< 0xDA
    int             date; ///< unix time
    float           cal[2][2][2]; ///< [channel 0:1][low/high gain 0:1][0 multiplicative : 1 additive]
    unsigned char   lin[2][34]; ///< [0:32] libearity calibration, 33 number of averages used during calibration, AD9717 pg 45
    char            log[22]; ///< BT log: Reference Board SN
    unsigned char   crc; ///< to generate: init 0 and -= 127 bytes; the checksum of the structure should be 0
}  __attribute__((__packed__)) CALIBECLYPSEDAC;

/**
 * Class containing functionality for ZMODDAC1411.
 */
class ZMODDAC1411: public ZMOD {
private:
protected:

public:
	ZMODDAC1411(uintptr_t baseAddress, uintptr_t dmaAddress, uintptr_t iicAddress, uintptr_t flashAddress, int dmaInterrupt);

	uint32_t* allocChannelsBuffer(size_t &length);
	void freeChannelsBuffer(uint32_t *buf, size_t length);
	uint32_t arrangeChannelData(uint8_t channel, uint16_t data);
	uint32_t arrangeSignedChannelData(uint8_t channel, int16_t data);

	void setOutputSampleFrequencyDivider(uint16_t val);
	uint8_t setData(uint32_t* buffer, size_t &length);
	void setGain(uint8_t channel, uint8_t gain);

	void start();
	void stop();
	void resetOutputCounter();
	void processInterrupt() override;

	int32_t computeCoefMult(float cg, uint8_t gain);
	int32_t computeCoefAdd(float ca, float cg, uint8_t gain);

	int readUserCalib() override;
	void setCalibValues(uint8_t channel, uint8_t gain, float valG, float valA);

	int32_t getSignedRawFromVolt(float voltValue, uint8_t gain);
};

#endif
